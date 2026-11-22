# Rover X 2026: ejecutor del área automática.
#
# Ejecuta las misiones de missions.py desde el nivel de inicio elegido, muestra el avance
# en la pantalla del CyberPi y se detiene si se acaba el tiempo, si falla una misión
# crítica o si se presiona B (abortar para pedir reinicio).

import hal
import config as cfg
import colors
import motion
import linefollow as lf
import gripper
import missions as ms

cards = []               # [(color, cm desde el inicio del escaneo)]
scan_end_cm = 0          # odometría al terminar el escaneo
results = []             # [(id, ok)]


def reset_state():
    global cards, scan_end_cm, results
    cards = []
    scan_end_cm = 0
    results = []


# ---------------------------------------------------------------- condiciones de línea
def make_cond(spec):
    kind = spec[0]
    if kind == "cross":
        return lf.at_intersection(spec[1])
    if kind == "color":
        return lf.at_color(spec[1])
    if kind == "cm":
        return lf.after_cm(spec[1])
    if kind == "obstacle":
        return lf.obstacle_within(spec[1])
    if kind == "end":
        return lf.line_end()
    if kind == "pitch":
        return lambda bits: hal.pitch() >= cfg.PITCH_ON_RAMP
    raise ValueError("condicion desconocida: %s" % kind)


# ---------------------------------------------------------------- misiones 11 y 12
def red_index(found):
    """Posición (0..5) de la tarjeta roja, o -1."""
    for i in range(len(found)):
        if found[i][0] == "red":
            return i
    return -1


def scan_cards():
    """Sigue la línea despacio y registra cada tarjeta de color (color + distancia)."""
    global cards, scan_end_cm
    cards = []
    lf.reset()
    hal.reset_odometer()
    t0 = hal.now()
    last = "white"
    same = 0
    while hal.now() - t0 < 15 and hal.odometer_cm() < cfg.CARD_SCAN_MAX_CM:
        if hal.abort_requested():
            break
        c = colors.read(cfg.CARD_PROBE)
        if c == last:
            same += 1
        else:
            last = c
            same = 1
        if same == cfg.COLOR_CONFIRM and colors.is_card_color(c):
            if not cards or cards[-1][0] != c or hal.odometer_cm() - cards[-1][1] > 6:
                cards.append((c, hal.odometer_cm()))
                colors.show(c)
                if len(cards) >= cfg.CARD_COUNT:
                    break
        lf.step(cfg.SPEED_LINE_SLOW)
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    scan_end_cm = hal.odometer_cm()
    return len(cards) > 0


def drop_red():
    i = red_index(cards)
    if i < 0:
        return False
    back = scan_end_cm - cards[i][1] + cfg.BALL_OFFSET_CM
    d = -1 if cfg.BAR_SIDE == "left" else 1
    ok = motion.straight(-back, cfg.SPEED_LINE_SLOW)
    ok = ok and motion.turn(90 * d)
    gripper.lift(cfg.LIFT_BAR)
    ok = ok and motion.straight(cfg.BAR_APPROACH_CM, cfg.SPEED_LINE_SLOW)
    gripper.open_claw()
    motion.straight(-cfg.BAR_APPROACH_CM, cfg.SPEED_LINE_SLOW)
    gripper.lift(cfg.LIFT_UP)
    motion.turn(-90 * d)
    motion.spin_to_line(-d, timeout=1.5)
    return ok


def stop_in(color):
    if not lf.follow_until(lf.at_color(color, ("L2", "L1", "R1", "R2")), cfg.SPEED_LINE, 15):
        return False
    motion.straight(cfg.GOAL_EXTRA_CM, cfg.SPEED_LINE_SLOW)
    hal.stop()
    return True


# ---------------------------------------------------------------- pasos
def run_step(step):
    kind = step[0]
    if kind == "line":
        return lf.follow_until(make_cond(step[1]), step[2], step[3], step[4])
    if kind == "straight":
        return motion.straight(step[1])
    if kind == "straight_slow":
        return motion.straight(step[1], cfg.SPEED_LINE_SLOW)
    if kind == "turn":
        return motion.turn(step[1])
    if kind == "spin":
        return motion.spin_to_line(step[1])
    if kind == "approach":
        target = step[1]
        return motion.straight_until(lambda: 0 < hal.distance_cm() <= target,
                                     cfg.SPEED_LINE_SLOW, 4.0)
    if kind == "climb":
        return motion.climb()
    if kind == "stairs":
        return motion.stairs()
    if kind == "claw":
        gripper.close_claw(step[1])
        return True
    if kind == "lift":
        gripper.lift(step[1])
        return True
    if kind == "hold":
        return gripper.hold()
    if kind == "detour":
        return motion.detour()
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
    """Pantalla de inicio: arriba/abajo cambia el nivel, A arranca. Sugiere el nivel por color."""
    level = detect_level()
    while True:
        hal.show("Rover X - nivel %d" % level, line=0)
        hal.show("Arr/Abj nivel  A=GO", line=1)
        if hal.button("up"):
            level = level % 4 + 1
            hal.wait(0.25)
        elif hal.button("down"):
            level = (level - 2) % 4 + 1
            hal.wait(0.25)
        elif hal.button("a"):
            return level
        hal.wait(0.02)
