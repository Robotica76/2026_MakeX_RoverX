## @file motion.py
#  @brief Movimientos con giroscopio y encoders (sin seguir la línea).
#  @details Todas las funciones de movimiento devuelven True si terminaron bien y False si se
#  acabó el tiempo o se pidió parar (botón A del CyberPi, cfg.ABORT_BUTTON).
#  Convención de giros: positivo = a la DERECHA (igual que hal.heading()).

import hal
import config as cfg
import geometry as geo
import colors


## @brief Signo de un número.
#  @param v Número.
#  @return -1 si es negativo, 1 si no.
def sign(v):
    if v < 0:
        return -1
    return 1


## @brief Giro relativo en el lugar.
#  @details Control proporcional (turn_kp) con potencia entre v_turn_min y v_turn_max; termina
#  cuando el error queda dentro de turn_tol durante TURN_SETTLE_S.
#  @param deg Grados a girar; positivo = derecha.
#  @param timeout s máximos (por defecto cfg.TURN_TIMEOUT_S).
#  @return True si llegó al ángulo.
def turn(deg, timeout=None):
    if timeout is None:
        timeout = cfg.TURN_TIMEOUT_S
    target = hal.heading() + deg
    t0 = hal.now()
    settled = None
    while hal.now() - t0 < timeout:
        if hal.abort_requested():
            break
        err = target - hal.heading()
        if abs(err) <= cfg.P["turn_tol"]:
            hal.stop()
            if settled is None:
                settled = hal.now()
            elif hal.now() - settled >= cfg.TURN_SETTLE_S:
                return True
        else:
            settled = None
            p = cfg.P["turn_kp"] * err
            p = sign(p) * hal.clamp(abs(p), cfg.P["v_turn_min"], cfg.P["v_turn_max"])
            hal.drive(p, -p)
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    return False


## @brief Giro relativo sobre UNA oruga (la otra detenida).
#  @details Avanza la oruga de afuera: al girar a la derecha la derecha queda quieta. Si la oruga
#  que se mueve patina (recortes del pasillo) el robot solo gira más lento: el centro del giro es
#  la oruga quieta y la posición final depende solo del ángulo (en un giro en el lugar una oruga
#  que patina corre el robot hasta 6 cm sin que la odometría lo vea).
#  @param deg Grados a girar; positivo = derecha.
#  @param speed % de potencia (por defecto v_turn_max); negativo = pivote hacia atrás.
#  @param timeout s máximos (por defecto el doble de TURN_TIMEOUT_S).
#  @return True si llegó al ángulo.
def pivot(deg, speed=None, timeout=None):
    if timeout is None:
        timeout = cfg.TURN_TIMEOUT_S * 2
    if speed is None:
        speed = cfg.P["v_turn_max"]
    target = hal.heading() + deg
    t0 = hal.now()
    settled = None
    while hal.now() - t0 < timeout:
        if hal.abort_requested():
            break
        err = target - hal.heading()
        if abs(err) <= cfg.P["turn_tol"]:
            hal.stop()
            if settled is None:
                settled = hal.now()
            elif hal.now() - settled >= cfg.TURN_SETTLE_S:
                return True
        else:
            settled = None
            p = hal.clamp(abs(cfg.P["turn_kp"] * err) * 2, cfg.P["v_turn_min"] * 1.5, abs(speed))
            # el giro sale de la diferencia izquierda - derecha (> 0 = a la derecha):
            # a la derecha hacia adelante mueve la izquierda, hacia atrás la derecha
            fwd = speed > 0
            if err > 0:
                hal.drive(p, 0) if fwd else hal.drive(0, -p)
            else:
                hal.drive(0, p) if fwd else hal.drive(-p, 0)
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    return False


## @brief Lleva un ángulo a -180..180.
#  @param deg Grados.
#  @return Grados equivalentes en -180..180.
def wrap180(deg):
    while deg > 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return deg


## @brief Gira hasta un rumbo absoluto por el camino más corto.
#  @param heading Rumbo objetivo (grados de hal.heading()).
#  @param timeout s máximos.
#  @return True si llegó.
def turn_to(heading, timeout=None):
    return turn(wrap180(heading - hal.heading()), timeout)


## @brief Un ciclo de avance sosteniendo un rumbo con el giroscopio.
#  @details El giro sale de la DIFERENCIA entre orugas (izquierda > derecha = a la derecha), igual
#  hacia adelante que hacia atrás. Antes se invertía la corrección al retroceder y el rumbo se iba
#  (en un retroceso largo sobre un recorte el robot daba vueltas).
#  @param speed % de potencia (negativo = atrás).
#  @param target Rumbo a sostener (grados de hal.heading()).
def _hold(speed, target):
    # el giro sale de la DIFERENCIA entre orugas (izquierda > derecha = a la derecha), igual
    # hacia adelante que hacia atrás. Antes se invertía la corrección al retroceder y el rumbo se
    # iba en vez de corregirse (en un retroceso largo sobre un recorte el robot daba vueltas).
    corr = cfg.P["head_kp"] * (target - hal.heading())
    hal.drive(speed + corr, speed - corr)


## @brief Avanza recto (manteniendo el rumbo) hasta que se cumpla una condición.
#  @param cond Función sin argumentos; el avance termina cuando devuelve True.
#  @param speed % de potencia (por defecto v_straight; negativo = atrás).
#  @param timeout s máximos.
#  @param target Rumbo a sostener (por defecto el actual).
#  @return True si terminó por cond().
def straight_until(cond, speed=None, timeout=5.0, target=None):
    if speed is None:
        speed = cfg.P["v_straight"]
    if target is None:
        target = hal.heading()
    t0 = hal.now()
    while hal.now() - t0 < timeout:
        if hal.abort_requested():
            break
        if cond():
            hal.stop()
            return True
        _hold(speed, target)
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    return False


## @brief Avanza o retrocede una distancia medida con encoders.
#  @details Sin target se sostiene el rumbo con el que empezó; con target, el error del giro
#  anterior y los golpes del piso se corrigen durante el avance.
#  @param cm Distancia: positiva = adelante, negativa = atrás.
#  @param speed % de potencia (por defecto v_straight; se usa su valor absoluto).
#  @param timeout s máximos (por defecto 1 + |cm| / 8).
#  @param target Rumbo (hal.heading) a sostener con el giroscopio.
#  @return True si recorrió la distancia.
def straight(cm, speed=None, timeout=None, target=None):
    if speed is None:
        speed = cfg.P["v_straight"]
    speed = abs(speed) * sign(cm)
    if timeout is None:
        timeout = 1.0 + abs(cm) / 8.0
    hal.reset_odometer()
    goal = abs(cm)
    return straight_until(lambda: abs(hal.odometer_cm()) >= goal, speed, timeout, target)


## @brief Sube una rampa o escalera y entra completamente al nivel.
#  @details Espera a inclinarse (pitch_ramp), luego a quedar plano (pitch_flat durante
#  FLAT_CONFIRM_S) y avanza extra_cm.
#  @param speed % de potencia (por defecto v_ramp).
#  @param timeout s máximos (por defecto cfg.RAMP_TIMEOUT_S).
#  @param extra_cm Avance final (por defecto geometry.ramp_extra()).
#  @return True si subió y entró.
def climb(speed=None, timeout=None, extra_cm=None):
    if speed is None:
        speed = cfg.P["v_ramp"]
    if timeout is None:
        timeout = cfg.RAMP_TIMEOUT_S
    if extra_cm is None:
        extra_cm = geo.ramp_extra()
    target = hal.heading()
    t0 = hal.now()
    climbing = False
    flat_since = None
    while hal.now() - t0 < timeout:
        if hal.abort_requested():
            hal.stop()
            return False
        p = hal.pitch()
        if not climbing:
            if p >= cfg.P["pitch_ramp"]:
                climbing = True
        elif p <= cfg.P["pitch_flat"]:
            if flat_since is None:
                flat_since = hal.now()
            elif hal.now() - flat_since >= cfg.FLAT_CONFIRM_S:
                hal.stop()
                return straight(extra_cm, cfg.P["v_straight"])
        else:
            flat_since = None
        _hold(speed, target)
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    return False


## @brief Sube la escalera (climb con v_stairs y STAIRS_TIMEOUT_S).
#  @param extra_cm Avance final después de quedar plano.
#  @return True si subió.
def stairs(extra_cm=None):
    return climb(cfg.P["v_stairs"], cfg.STAIRS_TIMEOUT_S, extra_cm)


## @brief ¿Alguna de las cuatro sondas ve ese color?
#  @details La línea pasa por encima de las zonas, así que las centrales leen negro; las
#  exteriores van a los lados (simulador).
#  @param name Nombre del color.
#  @return True si alguna sonda lo ve.
def sees_color(name):
    for pr in cfg.PROBES:
        if colors.read(pr) == name:
            return True
    return False


## @brief Sube hasta pasar la cima y avanza after_cm sin detenerse.
#  @details Para cuando arriba no hay espacio para esperar a quedar plano (nivel 4: la torre está
#  a ~20 cm de la escalera). La cima se detecta cuando el pitch bajó CREST_DROP grados desde su
#  máximo o, con color, cuando las sondas ven el color de la zona de arriba.
#  @param after_cm cm a avanzar después de la cima.
#  @param speed % de potencia (por defecto v_stairs).
#  @param timeout s máximos.
#  @param target Rumbo a sostener.
#  @param color Color de la zona de arriba (referencia exacta), o None.
#  @param arm_h Altura a la que sube el brazo al llegar (sin parar), o None.
#  @return True si pasó la cima y avanzó.
def climb_crest(after_cm, speed=None, timeout=None, target=None, color=None, arm_h=None):
    if speed is None:
        speed = cfg.P["v_stairs"]
    if timeout is None:
        timeout = cfg.STAIRS_TIMEOUT_S
    if target is None:
        target = hal.heading()
    t0 = hal.now()
    top = 0                         # pitch máximo visto
    while hal.now() - t0 < timeout:
        if hal.abort_requested():
            hal.stop()
            return False
        p = hal.pitch()
        if p > top:
            top = p
        # cima: ya subió y el pitch bajó CREST_DROP grados desde su máximo (el frente está arriba;
        # esperar a quedar plano no sirve porque la cola sigue en la escalera)
        if color is not None:
            # referencia exacta: las sondas centrales ven el color de la zona de arriba
            if top >= cfg.P["pitch_ramp"] and sees_color(color):
                if arm_h is not None:      # el brazo se mueve mientras sigue avanzando (sin parar)
                    hal.servo(cfg.SERVO_ARM, geo.arm_servo(arm_h))
                return straight(after_cm, cfg.P["v_ramp"], target=target)
        elif top >= cfg.P["pitch_ramp"] and p <= top - cfg.CREST_DROP:
            return straight(after_cm, cfg.P["v_ramp"], target=target)   # aún en pendiente: con fuerza
        _hold(speed, target)
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    return False


## @brief Gira en el lugar hasta que las sondas centrales vean línea.
#  @param direction 1 = derecha, -1 = izquierda.
#  @param timeout s máximos.
#  @param leave True: si ya está sobre una línea, primero sale de ella (para buscar OTRA línea).
#  False: si ya ve la línea, termina (tras un giro en un cruce las sondas ya quedan sobre la nueva).
#  @return True si encontró la línea.
def spin_to_line(direction=1, timeout=3.0, leave=True):
    t0 = hal.now()
    # salir primero de la línea actual si ya estamos encima
    while leave and hal.now() - t0 < timeout * 0.3:
        b = hal.line_bits()
        if not (b["L1"] or b["R1"]):
            break
        hal.drive(direction * cfg.P["v_turn_min"] * 1.5, -direction * cfg.P["v_turn_min"] * 1.5)
        hal.wait(cfg.LOOP_DT)
    while hal.now() - t0 < timeout:
        if hal.abort_requested():
            break
        b = hal.line_bits()
        if b["L1"] or b["R1"]:
            hal.stop()
            return True
        hal.drive(direction * cfg.P["v_turn_min"] * 1.5, -direction * cfg.P["v_turn_min"] * 1.5)
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    return False


## @brief ¿Las sondas centrales (L1 o R1) ven línea?
#  @return True si alguna ve negro.
def on_line():
    b = hal.line_bits()
    return b["L1"] or b["R1"]


## @brief Avanza recto (con giroscopio) hasta que las sondas centrales vean una línea.
#  @param speed % de potencia (por defecto v_slow).
#  @param max_cm cm máximos.
#  @param target Rumbo a sostener.
#  @return True si encontró la línea.
def seek_line(speed=None, max_cm=60, target=None):
    if speed is None:
        speed = cfg.P["v_slow"]
    hal.reset_odometer()
    t = 1.0 + max_cm / 8.0

    found = [False]

    def cond():
        if on_line():
            found[0] = True         # la lectura que detuvo el avance (releerla con ruido fallaba)
            return True
        return abs(hal.odometer_cm()) >= max_cm
    straight_until(cond, speed, t, target)
    return found[0]


## @brief Avanza recto hasta SALIR de una zona de color.
#  @details Referencia fija para empezar una maniobra sin importar dónde quedó o se colocó el
#  robot dentro de la zona. Pide COLOR_CONFIRM lecturas seguidas fuera de la zona.
#  @param color Color de la zona.
#  @param speed % de potencia (por defecto v_slow).
#  @param max_cm cm máximos.
#  @param target Rumbo a sostener.
#  @return True si salió de la zona.
def drive_to_edge(color, speed=None, max_cm=30, target=None):
    if speed is None:
        speed = cfg.P["v_slow"]
    hal.reset_odometer()
    state = {"seen": False, "out": 0, "ok": False}

    def cond():
        inside = sees_color(color)
        if inside:
            state["seen"] = True
            state["out"] = 0
        elif state["seen"]:
            state["out"] += 1
            if state["out"] >= cfg.COLOR_CONFIRM:
                state["ok"] = True
                return True
        return abs(hal.odometer_cm()) >= max_cm
    straight_until(cond, speed, 1.0 + max_cm / 6.0, target)
    return state["ok"]


## @brief Avanza recto hasta una línea que cruza el camino.
#  @details Las DOS sondas exteriores deben ver negro fuerte (cross_th), cada una con memoria
#  (pueden verla en distinto momento si la línea no es perpendicular), a menos de cross_win cm una
#  de otra. No es seguir la línea: sirve de referencia para corregir la distancia. Las marcas
#  verdes y el tope no cuentan.
#  @param speed % de potencia (por defecto v_slow).
#  @param max_cm cm máximos.
#  @param target Rumbo a sostener.
#  @return True si encontró el cruce.
def drive_to_cross(speed=None, max_cm=15, target=None):
    if speed is None:
        speed = cfg.P["v_slow"]
    hal.reset_odometer()
    st = {"found": False, "l": None, "r": None}

    def cond():
        _, bits = hal.line_read()
        odo = hal.odometer_cm()
        # con memoria (como los cruces): cada exterior puede ver la línea en distinto momento
        # si la línea no es perpendicular al camino (antes se pedían las dos a la vez)
        if bits.get("L2x", bits["L2"]):
            st["l"] = odo
        if bits.get("R2x", bits["R2"]):
            st["r"] = odo
        if st["l"] is not None and st["r"] is not None and abs(st["l"] - st["r"]) <= cfg.P["cross_win"]:
            st["found"] = True
            return True
        return abs(odo) >= max_cm
    straight_until(cond, speed, 1.0 + max_cm / 6.0, target)
    return st["found"]


## @brief Rodea un obstáculo fijo (torre) y vuelve a la línea.
#  @details Distancias desde geometry.py (detour_out, detour_pass). Al final gira hacia el mismo
#  lado del rodeo para seguir en el rumbo original.
#  @param side -1 = por la izquierda, 1 = por la derecha.
#  @return True si volvió a la línea.
def detour(side=None):
    if side is None:
        side = -1
    start = hal.heading()
    ok = turn(90 * side)
    ok = ok and straight(geo.detour_out())
    ok = ok and turn_to(start)
    ok = ok and straight(geo.detour_pass())
    ok = ok and turn(-90 * side)
    if not ok:
        return False
    if not straight_until(on_line, cfg.P["v_slow"], timeout=4.0):
        return False
    straight(geo.turn_advance(), cfg.P["v_slow"])   # centro de giro sobre la línea
    # quedó perpendicular a la línea; para seguir en el rumbo original se gira hacia el mismo
    # lado del rodeo (antes giraba al revés y regresaba hacia el obstáculo: lo mostró el simulador)
    return spin_to_line(side, leave=False)
