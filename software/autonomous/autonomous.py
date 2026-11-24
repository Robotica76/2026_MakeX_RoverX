## @file autonomous.py
#  @brief Ejecutor del área automática: corre las misiones de missions.py.
#  @details Ejecuta las misiones desde el nivel de inicio elegido (un TRAMO), muestra el avance en
#  la pantalla del CyberPi y se detiene al terminar el tramo, si se acaba el tiempo, si falla una
#  misión crítica o si se presiona A (cfg.ABORT_BUTTON: parar para pedir reinicio).
#  Cada misión es una lista de PASOS (tuplas); run_step() ejecuta uno. La lista de pasos y su
#  formato están en missions.py. También tiene la pantalla de inicio (choose_level) y las misiones
#  de tarjetas de colores (scan_cards, drop_red) y meta (stop_in).

import hal
import config as cfg
import geometry as geo
import colors
import motion
import linefollow as lf
import gripper
import missions as ms
import odometry as odo

cards = []               # [(color, odometría absoluta al detectarla)]
results = []             # [(id, ok)]
last_advance = 0         # cm del último acercamiento (para regresar lo mismo con "retreat")
heading_ref = 0          # rumbo de mapa = hal.heading() + heading_ref (paso "ref")
goal_entry = None        # odometría al entrar a la zona de la meta (lectura de tarjetas)
run_level = 1            # nivel en el que arrancó el autónomo (o el del último reinicio)
current_level = 1        # nivel de la misión en curso
pass_skipped = set()     # misiones saltadas en la pasada actual (ya hechas antes de un reinicio)
pass_ran = set()         # misiones corridas en la pasada actual
placed = True            # el robot recién fue colocado a mano (arranque o reinicio)
next_level = 1           # nivel que conviene elegir después del tramo
completed = set()        # misiones logradas en este encendido (se saltan al repetir un tramo)


## @brief Borra todo lo que una corrida anterior pudo dejar.
#  @details Tarjetas, resultados, último acercamiento, referencia de rumbo y estado del seguidor.
#  Al relanzar el autónomo sin apagar el robot, una referencia vieja movía el primer giro
#  (simulador).
def reset_state():
    global cards, results, last_advance, heading_ref, goal_entry
    goal_entry = None
    cards = []
    results = []
    last_advance = 0
    heading_ref = 0
    lf.reset()
    lf.last_junction = None
    lf.last_edge = None
    lf._after_turn = False


## @brief Valor de un argumento de paso.
#  @details En las misiones un valor puede ser un número o el NOMBRE de un parámetro de cfg.P; el
#  nombre se lee al ejecutar, así lo que se cambie en el menú se aplica sin recargar el programa.
#  @param x Número o nombre de parámetro (str).
#  @return El número, o cfg.P[x].
def val(x):
    if isinstance(x, str):
        return cfg.P[x]
    return x


# ---------------------------------------------------------------- condiciones de línea
## @brief Convierte la condición de un paso "line" en una función de parada.
#  @details ("junction", n, tipo), ("color", nombre), ("color_cm", color, cm), ("cm", d),
#  ("leave", color), ("obstacle", cm), ("end",), ("pitch",), ("head", rumbo, tolerancia).
#  @param spec Tupla de la condición.
#  @return Función cond(bits, junction) para linefollow.follow_until().
def make_cond(spec):
    kind = spec[0]
    if kind == "junction":
        return lf.at_junction(spec[1], spec[2])
    if kind == "color":
        # las marcas de color van JUNTO a la línea: se revisan las cuatro sondas
        return lf.at_color(spec[1], cfg.PROBES)
    if kind == "color_cm":
        return lf.color_then_cm(spec[1], val(spec[2]))
    if kind == "cm":
        return lf.after_cm(val(spec[1]))
    if kind == "leave":
        return lf.leave_color(spec[1])
    if kind == "obstacle":
        return lf.obstacle_within(val(spec[1]))       # solo con ultrasónico
    if kind == "end":
        return lf.line_end()
    if kind == "pitch":
        return lf.pitch_above(cfg.P["pitch_ramp"])
    if kind == "head":
        # ("head", rumbo de mapa, tolerancia): en una curva de la línea, parar al pasar por ese
        # rumbo (p. ej. a media curva, antes de que el seguidor se abra hacia un muro)
        target, tol = spec[1], spec[2]
        return lambda bits, j: abs(motion.wrap180(target - heading_ref - hal.heading())) < tol
    raise ValueError("condicion desconocida: %s" % kind)


# ---------------------------------------------------------------- misiones 11 y 12
## @brief Posición de la tarjeta roja en la lista leída.
#  @param found Lista [(color, odometría)].
#  @return Índice 0..5, o -1 si no hay roja.
def red_index(found):
    for i in range(len(found)):
        if found[i][0] == "red":
            return i
    return -1


## @brief Registra una tarjeta si no es una ya vista.
#  @details Mismo color a menos de 6 cm = la misma (la tarjeta mide 4.7 cm y al retroceder y volver
#  a pasar se lee en sus dos orillas). La muestra en los LEDs y la pantalla.
#  @param c Color leído.
#  @param odo Odometría absoluta (cm) al leerla.
#  @return True si era nueva.
def _note_card(c, odo):
    for c0, o0 in cards:
        if c0 == c and abs(o0 - odo) < 6:
            return False
    cards.append((c, odo))
    colors.show(c)
    return True


## @brief Retrocede recto leyendo tarjetas.
#  @details Al volver a la línea después de rodear la torre, la sonda ya pasó la primera tarjeta.
#  @param back_cm cm a retroceder.
def _scan_back(back_cm):
    start = hal.odo_abs()
    h0 = hal.heading()
    t0 = hal.now()
    last, same = "white", 0
    v = cfg.P["v_slow"]
    while start - hal.odo_abs() < back_cm and hal.now() - t0 < 4:
        if hal.abort_requested():
            break
        c = colors.read(cfg.CARD_PROBE)
        if c == last:
            same += 1
        else:
            last, same = c, 1
        if same == cfg.COLOR_CONFIRM and colors.is_card_color(c):
            _note_card(c, hal.odo_abs())
        corr = hal.clamp(cfg.P["head_kp"] * (h0 - hal.heading()), -15, 15)
        hal.drive(-v + corr, -v - corr)
        hal.wait(cfg.LOOP_DT)
    hal.stop()


## @brief Misión 11: sigue la línea despacio y registra cada tarjeta de color.
#  @details Para al ver la X azul de la meta (azul por más de GOAL_RUN_CM seguidos; una tarjeta
#  mide ~4.7 cm) o al recorrer CARD_SCAN_MAX_CM. El azul de la meta no cuenta como tarjeta.
#  @param back_cm Antes, retrocede eso leyendo (tarjetas que quedaron detrás de la sonda).
#  @return True si leyó al menos una tarjeta.
def scan_cards(back_cm=0):
    global cards
    cards = []
    if back_cm > 0:
        _scan_back(back_cm)
    lf.reset()
    start = hal.odo_abs()
    t0 = hal.now()
    last = "white"
    same = 0
    goal_from = None
    while hal.now() - t0 < 15 and hal.odo_abs() - start < cfg.CARD_SCAN_MAX_CM:
        # la X azul de la meta: azul por más de GOAL_RUN_CM seguidos (una tarjeta mide ~4.7 cm).
        # Sin esto seguía de largo y se salía de la arena (lo mostró el simulador).
        if motion.sees_color(cfg.GOAL_COLOR):
            if goal_from is None:
                goal_from = hal.odo_abs()
            elif hal.odo_abs() - goal_from >= cfg.GOAL_RUN_CM:
                global goal_entry
                goal_entry = goal_from
                break
        else:
            goal_from = None
        if hal.abort_requested():
            break
        c = colors.read(cfg.CARD_PROBE)
        if c == last:
            same += 1
        else:
            last = c
            same = 1
        odo = hal.odo_abs()
        if same == cfg.COLOR_CONFIRM and colors.is_card_color(c):
            # no se para al contar CARD_COUNT: la última tarjeta (azul) está junto a la meta azul
            # y parar encima de ella hacía que stop_in la tomara por la meta
            _note_card(c, odo)
        lf.step(cfg.P["v_slow"])
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    if goal_entry is not None and goal_entry >= start:   # meta confirmada en esta lectura:
        cards = [k for k in cards if k[1] < goal_entry - 1]  # su azul no es una tarjeta
    cards.sort(key=lambda k: k[1])          # en orden a lo largo de la línea (odometría)
    return len(cards) > 0


## @brief Misión 12 (omitida por ahora): deja la pelota en la casilla de la tarjeta roja.
#  @details Se coloca frente a la tarjeta roja, gira hacia la barra (bar_side), baja la garra a
#  h_bar, avanza bar_reach, suelta y regresa a la línea.
#  @return True si la dejó.
def drop_red():
    i = red_index(cards)
    if i < 0:
        return False
    side = cfg.P["bar_side"]
    ok = motion.straight(geo.card_to_center(cards[i][1], hal.odo_abs()), cfg.P["v_slow"])
    ok = ok and motion.turn(90 * side)
    gripper.arm_to(cfg.P["h_bar"])
    ok = ok and motion.straight(cfg.P["bar_reach"], cfg.P["v_slow"])
    gripper.open_claw()
    motion.straight(-cfg.P["bar_reach"], cfg.P["v_slow"])
    gripper.arm_to(cfg.P["h_carry"])
    motion.turn(-90 * side)
    motion.spin_to_line(-side, timeout=1.5)
    return ok


## @brief Misión 13: para completamente dentro de una zona (meta).
#  @details Avanza zone_extra() contado desde que las sondas ENTRARON a la zona; si la lectura de
#  tarjetas ya entró, no vuelve a contar desde aquí (antes se pasaba y salía por el otro lado).
#  @param color Color de la zona.
#  @return True si quedó dentro.
def stop_in(color):
    global goal_entry
    if goal_entry is None:
        start = {"odo": None}

        def cond(bits, j):
            if motion.sees_color(color):
                start["odo"] = hal.odo_abs()
                return True
            return False
        if not lf.follow_until(cond, cfg.P["v_line"], 15):
            return False
        goal_entry = start["odo"]
    rest = geo.zone_extra() - (hal.odo_abs() - goal_entry)
    if rest > 0.5:
        motion.straight(rest, cfg.P["v_slow"])
    hal.stop()
    return True


## @brief Avance de un acercamiento; lo guarda para que "retreat" regrese lo mismo.
#  @param cm cm a avanzar.
#  @return True si recorrió la distancia.
def _advance(cm):
    global last_advance
    last_advance = cm
    return motion.straight(cm, cfg.P["v_slow"])


## @brief Deja la garra sobre el centro de un pedestal medido desde el último cruce.
#  @param h Altura de la garra (número o nombre de parámetro).
#  @param from_cross cm del cruce al centro del pedestal (número o nombre de parámetro).
#  @return True si avanzó.
def approach_pedestal(h, from_cross):
    traveled = 0
    if lf.last_junction is not None:
        traveled = hal.odo_abs() - lf.last_junction[1]
    return _advance(geo.pedestal_advance(val(h), val(from_cross), traveled))


## @brief Deja la garra sobre un objeto al frente.
#  @param h Altura de la garra.
#  @param dist cm del centro de giro al centro del objeto.
#  @return True si avanzó.
def approach_object(h, dist):
    return _advance(geo.reach_advance(val(dist), val(h)))


## @brief Sigue la línea hasta quedar a side_margin de la torre (desde el borde de la zona verde).
#  @return True si llegó.
def to_tower():
    traveled = 0
    if lf.last_edge is not None:
        traveled = hal.odo_abs() - lf.last_edge
    adv = geo.tower_advance(traveled)
    if adv < 0:                         # ya está más cerca de lo debido: retroceder recto
        return motion.straight(adv, cfg.P["v_slow"])
    if adv < 0.5:
        return True
    return lf.follow_until(lf.after_cm(adv), cfg.P["v_slow"], 8)


RAMP_FLAT_DEG = 6.0     # al pie, en plano: corregir el desvío lateral antes de subir
RAMP_MAX_DEG = 1.5      # en la rampa angosta solo sostener: torcerse 3° saca una esquina 0.5 cm


## @brief Rumbo de mapa -> valor de hal.heading() al que hay que llegar por el camino corto.
#  @details hal.heading() acumula vueltas; sin normalizar, una recta podía "sostener" un rumbo a
#  360° del real y torcerse (simulador).
#  @param deg Rumbo de mapa (0 este, 90 sur, 180 oeste, -90 norte).
#  @return Grados en la escala de hal.heading().
def map_target(deg):
    now = hal.heading()
    return now + motion.wrap180(deg - heading_ref - now)


# ---------------------------------------------------------------- pasos
## @brief Ejecuta UN paso de una misión.
#  @details El primer elemento de la tupla dice qué hacer: "line", "jturn", "straight", "center",
#  "turn", "ref", "head", "drive", "pose", "square", "track_line", "diag", "track", "back_to",
#  "ramp_up", "fix_line", "bias", "fix", "pivot", "goto", "edge", "cross", "seek", "spin",
#  "approach_ped", "approach_obj", "retreat", "to_tower", "climb", "line_ref", "if_placed",
#  "if_skipped", "if_ran", "if_start", "crest", "stairs", "claw", "arm", "hold", "detour",
#  "cards", "drop_red", "stop_in", "wait". El formato de cada uno está en missions.py.
#  @param step Tupla del paso.
#  @return True si el paso terminó bien.
def run_step(step):
    global heading_ref
    kind = step[0]
    if kind == "line":
        green = len(step) > 4 and step[4]
        return lf.follow_until(make_cond(step[1]), val(step[2]), step[3], green)
    if kind == "jturn":
        return lf.turn_at_junction(step[1])
    if kind == "straight":
        speed = val(step[2]) if len(step) > 2 else cfg.P["v_straight"]
        return motion.straight(val(step[1]), speed)
    if kind == "center":
        return motion.straight(geo.turn_advance(), cfg.P["v_slow"])
    if kind == "turn":
        return motion.turn(step[1])
    if kind == "ref":
        # "ahora miro hacia step[1] grados de mapa" (0 = este, 90 = sur; positivo = horario).
        # Se usa en una recta conocida para que los giros "head" no arrastren el error de rumbo.
        # ("ref", rumbo, nivel): solo si el autónomo ARRANCÓ en ese nivel (robot colocado a mano
        # en la zona de inicio con ese rumbo).
        if len(step) > 2 and step[2] != run_level:
            return True
        heading_ref = step[1] - hal.heading()
        return True
    if kind == "head":
        return motion.turn_to(map_target(step[1]))
    if kind == "drive":
        # ("drive", rumbo, cm[, vel]): girar al rumbo de mapa y avanzar SOSTENIENDO ese rumbo
        # con el IMU (zona de obstáculos: no se arrastra el error del giro)
        target = map_target(step[1])
        speed = val(step[3]) if len(step) > 3 else cfg.P["v_slow"]
        motion.turn_to(target)
        return motion.straight(val(step[2]), speed, target=target)
    if kind == "pose":
        # ("pose", x, y, rumbo): posición conocida en el mapa (cm; x este, y sur) -> odometría
        # rumbo None: el que ya trae ("ref" anterior + giroscopio), solo se fija la posición
        # x o y None: se queda la que trae la odometría (p. ej. solo actualizar el rumbo tras line_ref)
        hdg = step[3] if step[3] is not None else heading_ref + hal.heading()
        ox, oy, _ = odo.pose()
        odo.set_pose(ox if step[1] is None else step[1], oy if step[2] is None else step[2], hdg)
        heading_ref = hdg - hal.heading()
        return True
    if kind == "square":
        # ("square", rumbo, segundos, coord, valor[, vel]): avanzar despacio DURANTE ese tiempo
        # hasta apoyar las orugas en un muro (rumbo perpendicular a él) y fijar esa coordenada de
        # la odometría ("x" o "y") y el rumbo. Por tiempo y no por distancia: con patinaje los
        # encoders cuentan de más y el robot se quedaba antes del muro (lo mostró el simulador).
        # 7o elemento opcional: valor de la OTRA coordenada de la recta a seguir (si no, la actual)
        speed = val(step[5]) if len(step) > 5 and step[5] is not None else cfg.P["v_slow"]
        motion.turn_to(map_target(step[1]))
        x, y, _ = odo.pose()
        if len(step) > 6:
            if step[3] == "x":
                y = step[6]
            else:
                x = step[6]
        odo.track(x, y, step[1], speed, timeout=val(step[2]))   # sostiene rumbo y línea recta
        # parar AQUÍ y esperar: si el paso siguiente empieza con los motores todavía empujando,
        # el robot sigue apretado contra el muro mientras ya gira
        hal.stop()
        hal.wait(0.3)
        x, y, _ = odo.pose()
        if step[3] == "x":
            x = step[4]
        else:
            y = step[4]
        odo.set_pose(x, y, step[1])
        heading_ref = step[1] - hal.heading()
        return True
    if kind == "track_line":
        # ("track_line", rumbo, coord, valor, max_cm[, vel[, max_deg]]): seguir la recta del mapa coord=valor
        # (p. ej. "y", 18.0 con rumbo 0 = hacia el este a esa altura) hasta que las sondas
        # centrales vean una línea. Absorbe el error a lo largo y corrige el lateral.
        speed = val(step[5]) if len(step) > 5 else cfg.P["v_slow"]
        motion.turn_to(map_target(step[1]))
        x, y, _ = odo.pose()
        if step[2] == "x":
            x = step[3]
        else:
            y = step[3]
        return odo.track(x, y, step[1], speed, timeout=2.0 + val(step[4]) / 5.0,
                         max_cm=val(step[4]), stop=motion.on_line,
                         max_deg=step[6] if len(step) > 6 else None)
    if kind == "diag":
        # ("diag", x0, y0, rumbo, max_cm, x_det, y_det[, vel[, head_kp]]): seguir la recta que pasa
        # por (x0, y0) con ese rumbo (cualquiera, no solo este/sur) corrigiendo rumbo y desvío
        # lateral, hasta que CUALQUIER sonda vea una línea. Ese punto de la línea es fijo (la
        # recta no cambia), así que ahí la odometría pasa a (x_det, y_det): el patinaje sobre los
        # recortes solo cambia CUÁNDO llega, no dónde.
        speed = val(step[7]) if len(step) > 7 else cfg.P["v_slow"]

        def seen():
            b = hal.line_bits()
            return any(b[pr] for pr in cfg.PROBES)
        kp0 = cfg.P["head_kp"]
        if len(step) > 8:
            cfg.P["head_kp"] = val(step[8])    # más firme: una oruga sobre un recorte la frena
        try:
            ok = odo.track(step[1], step[2], step[3], speed, timeout=2.0 + val(step[4]) / 4.0,
                           max_cm=val(step[4]), stop=seen)
        finally:
            cfg.P["head_kp"] = kp0
        if ok:
            _, _, h = odo.pose()
            odo.set_pose(step[5], step[6], h)
        return ok
    if kind == "track":
        # ("track", rumbo, coord, valor, cm[, vel]): como track_line pero sin parar en líneas
        speed = val(step[5]) if len(step) > 5 else cfg.P["v_slow"]
        motion.turn_to(map_target(step[1]))
        x, y, _ = odo.pose()
        if step[2] == "x":
            x = step[3]
        else:
            y = step[3]
        odo.track(x, y, step[1], speed, timeout=2.0 + val(step[4]) / 4.0, max_cm=val(step[4]))
        return True
    if kind == "back_to":
        # ("back_to", coord, valor[, vel[, rumbo, otra]]): retroceder hasta que la odometría
        # (encoders + giroscopio, corrige el patinaje de una oruga) llegue a esa coordenada. Con
        # rumbo y otra: por la recta del mapa otra (la otra coordenada) mirando a ese rumbo,
        # corrigiendo el desvío lateral (al retroceder sobre recortes el robot se corría 4 cm)
        speed = val(step[3]) if len(step) > 3 and step[3] is not None else cfg.P["v_slow"]
        i = 0 if step[1] == "x" else 1
        start = odo.pose()[i]
        goal = step[2]
        up = goal > start

        def there():
            v = odo.pose()[i]
            return v >= goal if up else v <= goal
        tmo = 2.0 + abs(goal - start) / 4.0
        if len(step) > 5:
            x, y, _ = odo.pose()
            if step[1] == "x":
                y = step[5]
            else:
                x = step[5]
            return odo.track(x, y, step[4], -abs(speed), timeout=tmo, stop=there)
        return motion.straight_until(there, -abs(speed), timeout=tmo)
    if kind == "ramp_up":
        # ("ramp_up", rumbo, x, color, extra_cm, vel, max_cm): subir una rampa angosta por la recta
        # del mapa x (giroscopio + odometría, sin seguir la línea: el seguidor oscilaba ±4.5° y una
        # oruga salía por la orilla) hasta ver ese color y luego extra_cm más, SIN detenerse
        hdg, x0, color, extra, speed, max_cm = step[1:7]
        speed = val(speed)
        _, y, _ = odo.pose()
        # en plano (antes de inclinarse) todavía se puede corregir el desvío lateral
        odo.track(x0, y, hdg, speed, timeout=4.0, max_cm=12, max_deg=RAMP_FLAT_DEG, halt=False,
                  stop=lambda: abs(hal.pitch()) > 4)
        _, y, _ = odo.pose()
        seen = odo.track(x0, y, hdg, speed, timeout=4.0 + val(max_cm) / 3.0, max_cm=val(max_cm),
                         stop=lambda: motion.sees_color(color), halt=False, max_deg=RAMP_MAX_DEG)
        if not seen:
            hal.stop()
            return False
        _, y, _ = odo.pose()
        odo.track(x0, y, hdg, speed, timeout=2.0 + val(extra) / 3.0, max_cm=val(extra),
                  max_deg=RAMP_MAX_DEG)
        return True
    if kind == "fix_line":
        # ("fix_line", coord, valor, signo): parado sobre una línea recta que está en coord=valor,
        # la desviación medida por las sondas da esa coordenada exacta. signo +1 si la izquierda
        # del robot es hacia +coord (p. ej. mirando al sur y la línea en x)
        po = cfg.P["probe_pitch"] / 2.0 + cfg.P.get("probe_outer", cfg.P["probe_pitch"])
        acc = 0.0
        n = 0
        for _ in range(5):
            err, bits = hal.line_read()
            if bits["L1"] or bits["R1"]:
                acc += err
                n += 1
            hal.wait(cfg.LOOP_DT)
        if n < 3:
            return True                       # no la ve: se queda la odometría
        err_cm = acc / n * po / 100.0         # err < 0: línea a la izquierda
        x, y, h = odo.pose()
        v = step[2] + step[3] * err_cm
        if step[1] == "x":
            x = v
        else:
            y = v
        odo.set_pose(x, y, h)
        return True
    if kind == "bias":
        # ("bias", cm): seguir la línea corrido; + = la línea a la derecha del robot. Para
        # centrar el robot en una rampa cuya línea no va al centro
        lf.bias_cm = step[1]
        return True
    if kind == "fix":
        # ("fix", coord, valor): la odometría está en esa coordenada (p. ej. al ver una línea
        # conocida); el resto de la pose se queda
        x, y, h = odo.pose()
        if step[1] == "x":
            x = step[2]
        else:
            y = step[2]
        odo.set_pose(x, y, h)
        return True
    if kind == "pivot":
        # ("pivot", rumbo[, "back"]): girar al rumbo de mapa sobre UNA oruga (ver motion.pivot);
        # "back" = la oruga que se mueve va hacia atrás
        speed = cfg.P["v_turn_max"]
        if "back" in step[2:]:
            speed = -speed
        return motion.pivot(motion.wrap180(map_target(step[1]) - hal.heading()), speed)
    if kind == "goto":
        # ("goto", x, y[, vel]): ir a ese punto del mapa con odometría encoders + giroscopio
        speed = val(step[3]) if len(step) > 3 else cfg.P["v_slow"]
        return odo.goto(step[1], step[2], speed)
    if kind == "edge":
        # ("edge", color, vel, max_cm[, rumbo]): recto hasta salir de la zona de ese color
        target = map_target(step[4]) if len(step) > 4 else None
        return motion.drive_to_edge(step[1], val(step[2]), val(step[3]), target)
    if kind == "cross":
        # ("cross", vel, max_cm[, rumbo]): recto hasta una línea que cruza el camino
        target = map_target(step[3]) if len(step) > 3 else None
        if target is not None:
            motion.turn_to(target)
        return motion.drive_to_cross(val(step[1]), val(step[2]), target)
    if kind == "seek":
        speed = val(step[1]) if len(step) > 1 else cfg.P["v_slow"]
        max_cm = val(step[2]) if len(step) > 2 else 60
        target = None
        if len(step) > 3:                      # rumbo de mapa opcional
            target = map_target(step[3])
            motion.turn_to(target)
        return motion.seek_line(speed, max_cm, target)
    if kind == "spin":
        return motion.spin_to_line(step[1])

    if kind == "approach_ped":
        return approach_pedestal(step[1], step[2])
    if kind == "approach_obj":
        return approach_object(step[1], step[2])
    if kind == "retreat":
        return motion.straight(-last_advance, cfg.P["v_slow"])
    if kind == "to_tower":
        return to_tower()
    if kind == "climb":
        return motion.climb()
    if kind == "line_ref":
        # ("line_ref", rumbo, cond, vel, timeout): sigue la línea hasta cond PROMEDIANDO el rumbo;
        # al terminar, ese promedio es "rumbo" de mapa. En una recta el seguidor oscila y una sola
        # lectura tenía ~4° de error; el promedio da la dirección de la línea.
        cond = make_cond(step[2])
        acc = [0.0, 0]

        def cond2(bits, j):
            acc[0] += hal.heading()
            acc[1] += 1
            return cond(bits, j)
        ok = lf.follow_until(cond2, val(step[3]), step[4])
        if acc[1] > 0:
            heading_ref = step[1] - acc[0] / acc[1]
        return ok
    if kind == "if_placed":
        if not placed:
            return True
        for sub in step[1]:
            if not run_step(sub):
                return False
        return True
    if kind in ("if_skipped", "if_ran"):
        # ("if_skipped", id, [pasos]): la misión id se saltó en esta pasada (ya estaba hecha y el
        # robot fue recolocado): pasos para llegar a donde ella lo habría dejado.
        # ("if_ran", id, [pasos]): la misión id sí se corrió en esta pasada.
        hit = step[1] in (pass_skipped if kind == "if_skipped" else pass_ran)
        if not hit:
            return True
        for sub in step[2]:
            if not run_step(sub):
                return False
        return True
    if kind == "if_start":
        if step[1] != run_level:
            return True
        for sub in step[2]:
            if not run_step(sub):
                return False
        return True
    if kind == "crest":
        # ("crest", cm[, rumbo[, color[, altura]]]): con color, la referencia es ver ese color
        # arriba; con altura, el brazo va a esa altura en ese momento sin detenerse;
        # con rumbo de mapa, sube sosteniéndolo con el IMU (escalera de
        # dos tramos: cada oruga debe ir sobre el suyo)
        target = None
        if len(step) > 2:
            target = map_target(step[2])
            motion.turn_to(target)
        color = step[3] if len(step) > 3 else None
        arm_h = val(step[4]) if len(step) > 4 else None
        return motion.climb_crest(val(step[1]), target=target, color=color, arm_h=arm_h)
    if kind == "stairs":
        return motion.stairs(val(step[1]) if len(step) > 1 else None)
    if kind == "claw":
        gripper.claw(val(step[1]))
        return True
    if kind == "arm":
        gripper.arm_to(val(step[1]))
        return True
    if kind == "hold":
        return gripper.hold()
    if kind == "detour":
        return motion.detour(step[1])
    if kind == "cards":
        # ("cards"[, retroceso cm]): primero retrocede leyendo (tarjetas que quedaron atrás)
        return scan_cards(val(step[1]) if len(step) > 1 else 0)
    if kind == "drop_red":
        return drop_red()
    if kind == "stop_in":
        return stop_in(step[1])
    if kind == "wait":
        hal.wait(step[1])
        return True
    raise ValueError("paso desconocido: %s" % kind)


## @brief Ejecuta todos los pasos de una misión.
#  @param m Dict de la misión (de missions.MISSIONS).
#  @return True si todos los pasos terminaron bien (False al primer paso que falla o si se pidió parar).
def run_mission(m):
    hal.show("M%d %s" % (m["id"], m["name"]), line=1)
    for step in m["steps"]:
        if hal.abort_requested():
            return False
        if not run_step(step):
            return False
    return True


## @brief Corre el TRAMO que empieza en start_level.
#  @details Con cfg.SEGMENTS el robot se detiene al terminar el tramo (misión con "end_level") o al
#  fallar una misión crítica; next_level queda con el nivel que conviene elegir después.
#  @param start_level Nivel elegido en la pantalla de inicio (1..4).
#  @param budget_s Tiempo máximo (por defecto cfg.AUTO_BUDGET_S).
#  @param t_start Momento de inicio (por defecto ahora).
#  @param skip Misiones ya logradas en este encendido: solo se saltan las de manipulación (las de
#  navegación, críticas, se repiten porque llevan al robot a donde empieza la siguiente).
#  @return Puntos estimados del tramo.
def run(start_level=1, budget_s=None, t_start=None, skip=None):
    global run_level, placed, next_level, current_level
    if budget_s is None:
        budget_s = cfg.AUTO_BUDGET_S
    if t_start is None:
        t_start = hal.now()
    reset_state()
    run_level = start_level
    next_level = start_level
    placed = True
    crit = {}
    for m in ms.MISSIONS:
        crit[m["id"]] = m["critical"]
    done = set()
    for mid in (skip or ()):
        if not crit.get(mid, True):
            done.add(mid)
    points = 0
    pass_skipped.clear()
    pass_ran.clear()
    for m in ms.MISSIONS:
        if m["level"] < start_level:
            continue
        if m["id"] in done:
            pass_skipped.add(m["id"])
            continue
        if hal.now() - t_start >= budget_s or hal.abort_requested():
            break
        pass_ran.add(m["id"])
        current_level = m["level"]
        ok = run_mission(m)
        placed = False
        results.append((m["id"], ok))
        hal.stop()
        if ok:
            done.add(m["id"])
            completed.add(m["id"])
            points += m["pts"]
            hal.leds(0, 80, 0)
            hal.show("Pts aprox: %d" % points, line=2)
            if m.get("end_level") and cfg.SEGMENTS:
                next_level = m["level"] + 1
                break
            if m is ms.MISSIONS[-1]:
                next_level = 5              # terminó todo: la pantalla vuelve a leer la zona
        else:
            hal.leds(80, 0, 0)
            if hal.abort_requested() or m["critical"]:
                break
    hal.stop()
    return points


# ---------------------------------------------------------------- elegir nivel de inicio
## @brief Nivel según el color de la zona de inicio bajo el robot.
#  @return 1..4 (rojo, amarillo, azul, verde); 1 si no se reconoce.
def detect_level():
    for probe in ("L1", "R1"):
        lvl = colors.zone_level(colors.read_stable(probe))
        if lvl:
            return lvl
    return 1


## @brief Pantalla de inicio del autónomo.
#  @details Joystick del CyberPi: elige el nivel (cfg.LEVEL_KEYS: arriba 1, abajo 2, izquierda 3,
#  derecha 4). B (cfg.START_BUTTON): arrancar el tramo. Centro (cfg.MENU_BUTTON): menú de ajustes.
#  @param level Nivel sugerido (None = leerlo del color de la zona).
#  @return Nivel elegido 1..4, o 0 = ir al menú de ajustes.
def choose_level(level=None):
    if level is None:
        level = detect_level()
    hal.wait(0.3)
    while True:
        hal.show("Nivel %d" % level, line=0)
        hal.show("^1 v2 <3 >4  %s=GO" % cfg.START_BUTTON.upper(), line=1)
        hal.show("%s=parar  centro=ajustes" % cfg.ABORT_BUTTON.upper(), line=2)
        for key in cfg.LEVEL_KEYS:
            if hal.button(key):
                level = cfg.LEVEL_KEYS[key]
        if hal.button(cfg.START_BUTTON):
            while hal.button(cfg.START_BUTTON):     # que no cuente como otro botón al soltarlo
                hal.wait(0.02)
            return level
        if hal.button(cfg.MENU_BUTTON):
            return 0
        hal.wait(0.02)
