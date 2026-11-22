# Rover X 2026: movimientos con giroscopio y encoders (sin línea).
#
# Todas las funciones devuelven True si terminaron bien y False si se acabó el tiempo
# o se pidió abortar (botón B del CyberPi).

import hal
import config as cfg
import geometry as geo
import colors


def sign(v):
    if v < 0:
        return -1
    return 1


def turn(deg, timeout=None):
    """Giro relativo en el lugar; positivo = derecha."""
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


def wrap180(deg):
    while deg > 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return deg


def turn_to(heading, timeout=None):
    """Gira hasta un rumbo absoluto por el camino más corto (antes podía dar la vuelta larga)."""
    return turn(wrap180(heading - hal.heading()), timeout)


def _hold(speed, target):
    corr = cfg.P["head_kp"] * (target - hal.heading())
    if speed < 0:
        corr = -corr
    hal.drive(speed + corr, speed - corr)


def straight_until(cond, speed=None, timeout=5.0, target=None):
    """Avanza recto (manteniendo el rumbo) hasta que cond() sea verdadero."""
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


def straight(cm, speed=None, timeout=None, target=None):
    """Avanza (cm > 0) o retrocede (cm < 0) una distancia medida con encoders.

    target: rumbo (hal.heading) que se sostiene con el giroscopio. Sin target se sostiene el
    rumbo con el que empezó; con target, el error del giro anterior y los golpes del piso se
    corrigen durante el avance."""
    if speed is None:
        speed = cfg.P["v_straight"]
    speed = abs(speed) * sign(cm)
    if timeout is None:
        timeout = 1.0 + abs(cm) / 8.0
    hal.reset_odometer()
    goal = abs(cm)
    return straight_until(lambda: abs(hal.odometer_cm()) >= goal, speed, timeout, target)


def climb(speed=None, timeout=None, extra_cm=None):
    """Sube una rampa o escalera: espera a inclinarse, luego a quedar plano, y avanza
    extra_cm para quedar completamente dentro del nivel."""
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


def stairs(extra_cm=None):
    return climb(cfg.P["v_stairs"], cfg.STAIRS_TIMEOUT_S, extra_cm)


def sees_color(name):
    """Alguna de las cuatro sondas ve ese color. La línea pasa por encima de las zonas, así que
    las centrales leen negro; las exteriores van a los lados (lo mostró el simulador)."""
    for pr in cfg.PROBES:
        if colors.read(pr) == name:
            return True
    return False


def climb_crest(after_cm, speed=None, timeout=None, target=None, color=None, arm_h=None):
    """Sube hasta pasar la cima (el pitch ya subió y empieza a bajar: el frente está arriba) y
    avanza after_cm medidos con encoder. Sirve cuando arriba no hay espacio para esperar a quedar
    plano (nivel 4: la torre está a ~20 cm de la escalera; lo mostró el simulador)."""
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


def spin_to_line(direction=1, timeout=3.0, leave=True):
    """Gira en el lugar (1 = derecha, -1 = izquierda) hasta que las sondas centrales vean línea.

    leave=True: si ya está sobre una línea, primero sale de ella (para buscar OTRA línea).
    leave=False: si ya ve la línea, termina (después de un giro en un cruce las sondas, que van
    cerca del centro de giro, ya quedan sobre la línea nueva)."""
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


def on_line():
    b = hal.line_bits()
    return b["L1"] or b["R1"]


def seek_line(speed=None, max_cm=60, target=None):
    """Avanza recto (con giroscopio) hasta que las sondas centrales vean una línea."""
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


def drive_to_edge(color, speed=None, max_cm=30, target=None):
    """Avanza recto (IMU) hasta SALIR de una zona de color: referencia fija para empezar una
    maniobra sin importar dónde quedó o se colocó el robot dentro de la zona."""
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


def drive_to_cross(speed=None, max_cm=15, target=None):
    """Avanza recto (IMU) hasta que las DOS sondas exteriores ven negro fuerte: una línea que
    cruza el camino. No es seguir la línea: solo sirve de referencia para corregir la distancia.
    Las marcas verdes y el tope no cuentan (cross_th). Devuelve True si encontró el cruce."""
    if speed is None:
        speed = cfg.P["v_slow"]
    hal.reset_odometer()
    found = [False]

    def cond():
        _, bits = hal.line_read()
        if bits.get("L2x", bits["L2"]) and bits.get("R2x", bits["R2"]):
            found[0] = True
            return True
        return abs(hal.odometer_cm()) >= max_cm
    straight_until(cond, speed, 1.0 + max_cm / 6.0, target)
    return found[0]


def detour(side=None):
    """Rodea un obstáculo fijo (torre, misión 10) y vuelve a buscar la línea.
    side: -1 = por la izquierda, 1 = por la derecha. Distancias desde geometry.py."""
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
