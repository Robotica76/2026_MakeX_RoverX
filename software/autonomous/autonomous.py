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


def reset_state():
    global cards, results
    cards = []
    results = []


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
        return lf.at_color(spec[1])
    if kind == "cm":
        return lf.after_cm(val(spec[1]))
    if kind == "obstacle":
        return lf.obstacle_within(val(spec[1]))
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


def approach_pedestal(h):
    target = geo.pedestal_us_target(val(h))
    return motion.straight_until(lambda: 0 < hal.distance_cm() <= target,
                                 cfg.P["v_slow"], 4.0)


# ---------------------------------------------------------------- pasos
def run_step(step):
    kind = step[0]
    if kind == "line":
        green = len(step) > 4 and step[4]
        return lf.follow_until(make_cond(step[1]), val(step[2]), step[3], green)
    if kind == "jturn":
        return lf.turn_at_junction(step[1])
    if kind == "straight":
        speed = val(step[2]) if len(step) > 2 else cfg.P["v_straight"]
        return motion.straight(val(step[1]), speed)
    if kind == "turn":
        return motion.turn(step[1])
    if kind == "spin":
        return motion.spin_to_line(step[1])
    if kind == "approach_ped":
        return approach_pedestal(step[1])
    if kind == "climb":
        return motion.climb()
    if kind == "stairs":
        return motion.stairs()
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
