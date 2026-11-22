# Rover X 2026: movimientos con giroscopio y encoders (sin línea).
#
# Todas las funciones devuelven True si terminaron bien y False si se acabó el tiempo
# o se pidió abortar (botón B del CyberPi).

import hal
import config as cfg
import geometry as geo


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


def turn_to(heading, timeout=None):
    return turn(heading - hal.heading(), timeout)


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


def stairs():
    return climb(cfg.P["v_stairs"], cfg.STAIRS_TIMEOUT_S)


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

    def cond():
        return on_line() or abs(hal.odometer_cm()) >= max_cm
    return straight_until(cond, speed, t, target) and on_line()


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
