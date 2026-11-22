# Rover X 2026: ejecutor del área automática.
#
# Ejecuta las misiones de missions.py desde el nivel de inicio elegido, muestra el avance
# en la pantalla del CyberPi y se detiene si se acaba el tiempo, si falla una misión
# crítica o si se presiona B (abortar para pedir reinicio).

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


def reset_state():
    """Todo lo que una corrida anterior pudo dejar (al relanzar el autónomo desde el menú sin
    apagar el robot, una referencia vieja movía el primer giro; lo mostró el simulador)."""
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


def val(x):
    """Número tal cual, o el valor actual del parámetro cfg.P con ese nombre."""
    if isinstance(x, str):
        return cfg.P[x]
    return x


# ---------------------------------------------------------------- condiciones de línea
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
    raise ValueError("condicion desconocida: %s" % kind)


# ---------------------------------------------------------------- misiones 11 y 12
def red_index(found):
    """Posición (0..5) de la tarjeta roja, o -1."""
    for i in range(len(found)):
        if found[i][0] == "red":
            return i
    return -1


def scan_cards():
    """Sigue la línea despacio y registra cada tarjeta de color (color + odometría)."""
    global cards
    cards = []
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
            if not cards or cards[-1][0] != c or odo - cards[-1][1] > 6:
                cards.append((c, odo))
                colors.show(c)
                if len(cards) >= cfg.CARD_COUNT:
                    break
        lf.step(cfg.P["v_slow"])
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    return len(cards) > 0


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


def stop_in(color):
    """Para completamente dentro de la zona: avanza zone_extra() contado desde que las sondas
    ENTRARON a la zona (si la lectura de tarjetas ya entró, no se vuelve a contar desde aquí;
    antes el robot se pasaba y salía por el otro lado: lo mostró el simulador)."""
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


def _advance(cm):
    global last_advance
    last_advance = cm
    return motion.straight(cm, cfg.P["v_slow"])


def approach_pedestal(h, from_cross):
    """Garra sobre el centro de un pedestal que está a from_cross cm del último cruce."""
    traveled = 0
    if lf.last_junction is not None:
        traveled = hal.odo_abs() - lf.last_junction[1]
    return _advance(geo.pedestal_advance(val(h), val(from_cross), traveled))


def approach_object(h, dist):
    """Garra sobre un objeto cuyo centro está a dist cm del centro de giro, al frente."""
    return _advance(geo.reach_advance(val(dist), val(h)))


def to_tower():
    """Sigue la línea hasta quedar a side_margin de la torre (referencia: borde de la zona verde)."""
    traveled = 0
    if lf.last_edge is not None:
        traveled = hal.odo_abs() - lf.last_edge
    adv = geo.tower_advance(traveled)
    if adv < 0:                         # ya está más cerca de lo debido: retroceder recto
        return motion.straight(adv, cfg.P["v_slow"])
    if adv < 0.5:
        return True
    return lf.follow_until(lf.after_cm(adv), cfg.P["v_slow"], 8)


def map_target(deg):
    """Rumbo de mapa -> valor de hal.heading() al que hay que llegar por el camino corto.
    hal.heading() acumula vueltas; sin normalizar, una recta podía "sostener" un rumbo a 360°
    del real y torcerse (lo mostró el simulador)."""
    now = hal.heading()
    return now + motion.wrap180(deg - heading_ref - now)


# ---------------------------------------------------------------- pasos
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
        odo.set_pose(step[1], step[2], step[3])
        heading_ref = step[3] - hal.heading()
        return True
    if kind == "square":
        # ("square", rumbo, segundos, coord, valor[, vel]): avanzar despacio DURANTE ese tiempo
        # hasta apoyar las orugas en un muro (rumbo perpendicular a él) y fijar esa coordenada de
        # la odometría ("x" o "y") y el rumbo. Por tiempo y no por distancia: con patinaje los
        # encoders cuentan de más y el robot se quedaba antes del muro (lo mostró el simulador).
        speed = val(step[5]) if len(step) > 5 else cfg.P["v_slow"]
        motion.turn_to(map_target(step[1]))
        x, y, _ = odo.pose()
        odo.track(x, y, step[1], speed, timeout=val(step[2]))   # sostiene rumbo y línea recta
        x, y, _ = odo.pose()
        if step[3] == "x":
            x = step[4]
        else:
            y = step[4]
        odo.set_pose(x, y, step[1])
        heading_ref = step[1] - hal.heading()
        return True
    if kind == "track_line":
        # ("track_line", rumbo, coord, valor, max_cm[, vel]): seguir la recta del mapa coord=valor
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
                         max_cm=val(step[4]), stop=motion.on_line)
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
        return scan_cards()
    if kind == "drop_red":
        return drop_red()
    if kind == "stop_in":
        return stop_in(step[1])
    if kind == "wait":
        hal.wait(step[1])
        return True
    raise ValueError("paso desconocido: %s" % kind)


def run_mission(m):
    hal.show("M%d %s" % (m["id"], m["name"]), line=1)
    for step in m["steps"]:
        if hal.abort_requested():
            return False
        if not run_step(step):
            return False
    return True


def run(start_level=1, budget_s=None, t_start=None, skip=None):
    """Corre el TRAMO que empieza en start_level. Devuelve los puntos estimados.

    Con cfg.SEGMENTS el robot se detiene al terminar el tramo (misión con "end_level") o al
    fallar una misión crítica; next_level queda con el nivel que conviene elegir después.
    skip: misiones ya logradas en este encendido; solo se saltan las de manipulación (las de
    navegación, críticas, se repiten porque llevan al robot a donde empieza la siguiente)."""
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
def detect_level():
    """Lee el color de la zona de inicio bajo el robot (rojo/amarillo/azul/verde)."""
    for probe in ("L1", "R1"):
        lvl = colors.zone_level(colors.read_stable(probe))
        if lvl:
            return lvl
    return 1


def choose_level(level=None):
    """Pantalla de inicio. Joystick: elige el nivel (cfg.LEVEL_KEYS: arriba 1, abajo 2,
    izquierda 3, derecha 4). START_BUTTON: arrancar el tramo. MANUAL_BUTTON: control manual.
    Centro del joystick: menú de ajustes. Devuelve el nivel, 0 = ajustes, -1 = manual."""
    if level is None:
        level = detect_level()
    hal.wait(0.3)
    while True:
        hal.show("Nivel %d" % level, line=0)
        hal.show("^1 v2 <3 >4  %s=GO" % cfg.START_BUTTON.upper(), line=1)
        hal.show("%s=manual  centro=ajustes" % cfg.MANUAL_BUTTON.upper(), line=2)
        for key in cfg.LEVEL_KEYS:
            if hal.button(key):
                level = cfg.LEVEL_KEYS[key]
        if hal.button(cfg.START_BUTTON):
            while hal.button(cfg.START_BUTTON):     # que no cuente como otro botón al soltarlo
                hal.wait(0.02)
            return level
        if hal.button(cfg.MENU_BUTTON):
            return 0
        if hal.button(cfg.MANUAL_BUTTON):
            while hal.button(cfg.MANUAL_BUTTON):
                hal.wait(0.02)
            return -1
        hal.wait(0.02)
