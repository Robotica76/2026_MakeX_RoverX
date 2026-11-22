# Rover X 2026: movimientos con giroscopio y encoders (sin línea).
#
# Todas las funciones devuelven True si terminaron bien y False si se acabó el tiempo
# o se pidió abortar (botón B del CyberPi).

import hal
import config as cfg


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
        if abs(err) <= cfg.TURN_TOL_DEG:
            hal.stop()
            if settled is None:
                settled = hal.now()
            elif hal.now() - settled >= cfg.TURN_SETTLE_S:
                return True
        else:
            settled = None
            p = cfg.TURN_KP * err
            p = sign(p) * hal.clamp(abs(p), cfg.SPEED_TURN_MIN, cfg.SPEED_TURN_MAX)
            hal.drive(p, -p)
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    return False


def turn_to(heading, timeout=None):
    return turn(heading - hal.heading(), timeout)


def _hold(speed, target):
    corr = cfg.HEADING_KP * (target - hal.heading())
    if speed < 0:
        corr = -corr
    hal.drive(speed + corr, speed - corr)


def straight_until(cond, speed=None, timeout=5.0, target=None):
    """Avanza recto (manteniendo el rumbo) hasta que cond() sea verdadero."""
    if speed is None:
        speed = cfg.SPEED_STRAIGHT
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


def straight(cm, speed=None, timeout=None):
    """Avanza (cm > 0) o retrocede (cm < 0) una distancia medida con encoders."""
    if speed is None:
        speed = cfg.SPEED_STRAIGHT
    speed = abs(speed) * sign(cm)
    if timeout is None:
        timeout = 1.0 + abs(cm) / 8.0
    hal.reset_odometer()
    goal = abs(cm)
    return straight_until(lambda: hal.odometer_cm() >= goal, speed, timeout)


def climb(speed=None, timeout=None, extra_cm=None):
    """Sube una rampa o escalera: espera a inclinarse, luego a quedar plano, y avanza
    extra_cm para quedar completamente dentro del nivel."""
    if speed is None:
        speed = cfg.SPEED_RAMP
    if timeout is None:
        timeout = cfg.RAMP_TIMEOUT_S
    if extra_cm is None:
        extra_cm = cfg.RAMP_EXTRA_CM
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
            if p >= cfg.PITCH_ON_RAMP:
                climbing = True
        elif p <= cfg.PITCH_FLAT:
            if flat_since is None:
                flat_since = hal.now()
            elif hal.now() - flat_since >= cfg.FLAT_CONFIRM_S:
                hal.stop()
                return straight(extra_cm, cfg.SPEED_STRAIGHT)
        else:
            flat_since = None
        _hold(speed, target)
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    return False


def stairs():
    return climb(cfg.SPEED_STAIRS, cfg.STAIRS_TIMEOUT_S)


def spin_to_line(direction=1, timeout=3.0):
    """Gira en el lugar (1 = derecha, -1 = izquierda) hasta que las sondas centrales vean línea."""
    t0 = hal.now()
    # salir primero de la línea actual si ya estamos encima
    while hal.now() - t0 < timeout * 0.3:
        b = hal.line_bits()
        if not (b["L1"] or b["R1"]):
            break
        hal.drive(direction * cfg.SPEED_TURN_MIN * 1.5, -direction * cfg.SPEED_TURN_MIN * 1.5)
        hal.wait(cfg.LOOP_DT)
    while hal.now() - t0 < timeout:
        if hal.abort_requested():
            break
        b = hal.line_bits()
        if b["L1"] or b["R1"]:
            hal.stop()
            return True
        hal.drive(direction * cfg.SPEED_TURN_MIN * 1.5, -direction * cfg.SPEED_TURN_MIN * 1.5)
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    return False


def detour(side=None):
    """Rodea un obstáculo fijo (torre, misión 10) y vuelve a buscar la línea."""
    if side is None:
        side = cfg.TOWER_SIDE
    d = -1 if side == "left" else 1
    start = hal.heading()
    ok = turn(90 * d)
    ok = ok and straight(cfg.TOWER_OUT_CM)
    ok = ok and turn_to(start)
    ok = ok and straight(cfg.TOWER_PASS_CM)
    ok = ok and turn(-90 * d)
    if not ok:
        return False

    def on_line():
        b = hal.line_bits()
        return b["L1"] or b["R1"]

    if not straight_until(on_line, cfg.SPEED_LINE_SLOW, timeout=4.0):
        return False
    straight(4, cfg.SPEED_LINE_SLOW)
    return spin_to_line(-d)
