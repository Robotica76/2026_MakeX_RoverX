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

cards = []               # [(color, odometría absoluta al detectarla)]
results = []             # [(id, ok)]
last_advance = 0         # cm del último acercamiento (para regresar lo mismo con "retreat")
heading_ref = 0          # rumbo de mapa = hal.heading() + heading_ref (paso "ref")
run_level = 1            # nivel en el que arrancó el autónomo


def reset_state():
    """Todo lo que una corrida anterior pudo dejar (al relanzar el autónomo desde el menú sin
    apagar el robot, una referencia vieja movía el primer giro; lo mostró el simulador)."""
    global cards, results, last_advance, heading_ref
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
    while hal.now() - t0 < 15 and hal.odo_abs() - start < cfg.CARD_SCAN_MAX_CM:
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
    if not lf.follow_until(lf.at_color(color, cfg.PROBES), cfg.P["v_line"], 15):
        return False
    motion.straight(geo.zone_extra(), cfg.P["v_slow"])
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
    if kind == "if_start":
        if step[1] != run_level:
            return True
        for sub in step[2]:
            if not run_step(sub):
                return False
        return True
    if kind == "crest":
        # ("crest", cm[, rumbo[, color]]): con color, la referencia es ver ese color arriba;
        # con rumbo de mapa, sube sosteniéndolo con el IMU (escalera de
        # dos tramos: cada oruga debe ir sobre el suyo)
        target = None
        if len(step) > 2:
            target = map_target(step[2])
            motion.turn_to(target)
        color = step[3] if len(step) > 3 else None
        return motion.climb_crest(val(step[1]), target=target, color=color)
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


def run(start_level=1, budget_s=None, t_start=None):
    """Ejecuta el autónomo. Devuelve los puntos estimados de las misiones logradas."""
    if budget_s is None:
        budget_s = cfg.AUTO_BUDGET_S
    if t_start is None:
        t_start = hal.now()
    reset_state()
    global run_level
    run_level = start_level
    points = 0
    for m in ms.MISSIONS:
        if m["level"] < start_level:
            continue
        if hal.now() - t_start >= budget_s or hal.abort_requested():
            break
        ok = run_mission(m)
        results.append((m["id"], ok))
        hal.stop()
        if ok:
            points += m["pts"]
            hal.leds(0, 80, 0)
        else:
            hal.leds(80, 0, 0)
            if m["critical"] or hal.abort_requested():
                break
        hal.show("Pts aprox: %d" % points, line=2)
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


def choose_level():
    """Pantalla de inicio. Arriba/abajo: nivel. A: arrancar. Derecha: menú de ajustes.
    Devuelve el nivel elegido, o 0 si se pidió el menú de ajustes."""
    level = detect_level()
    hal.wait(0.3)
    while True:
        hal.show("Rover X - nivel %d" % level, line=0)
        hal.show("Arr/Abj nivel  A=GO", line=1)
        hal.show("Der = ajustes", line=2)
        if hal.button("up"):
            level = level % 4 + 1
            hal.wait(0.25)
        elif hal.button("down"):
            level = (level - 2) % 4 + 1
            hal.wait(0.25)
        elif hal.button("right"):
            return 0
        elif hal.button("a"):
            return level
        hal.wait(0.02)
