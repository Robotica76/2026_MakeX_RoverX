# Rover X 2026: seguidor de línea PID con intersecciones y marcas verdes.

import hal
import config as cfg
import colors
import motion

_last_err = 0
_integral = 0
_pending_turn = 0        # -1 izq, 1 der, 2 media vuelta (marcas verdes)


def reset():
    global _last_err, _integral, _pending_turn
    _last_err = 0
    _integral = 0
    _pending_turn = 0


def correction(err, last_err, integral):
    """Corrección PID (función pura para poder probarla en PC)."""
    return cfg.LINE_KP * err + cfg.LINE_KI * integral + cfg.LINE_KD * (err - last_err)


def step(speed):
    """Un ciclo de control. Devuelve el error leído."""
    global _last_err, _integral
    err = hal.line_offset()
    _integral = hal.clamp(_integral + err, -cfg.LINE_I_MAX, cfg.LINE_I_MAX)
    c = correction(err, _last_err, _integral)
    _last_err = err
    # línea a la izquierda (err < 0) -> frenar la oruga izquierda
    hal.drive(speed + c, speed - c)
    return err


def is_intersection(bits):
    return bits["L2"] and bits["R2"] or (bits["L2"] and bits["L1"] and bits["R1"]) \
        or (bits["R2"] and bits["R1"] and bits["L1"])


def sees_line(bits):
    return bits["L2"] or bits["L1"] or bits["R1"] or bits["R2"]


def _check_green():
    """Marca verde antes de una intersección = girar hacia ese lado al llegar."""
    global _pending_turn
    left = colors.read("L2") == "green"
    right = colors.read("R2") == "green"
    if left and right:
        _pending_turn = 2
    elif left:
        _pending_turn = -1
    elif right:
        _pending_turn = 1


def _take_pending_turn():
    global _pending_turn
    t = _pending_turn
    _pending_turn = 0
    motion.straight(6, cfg.SPEED_LINE_SLOW)       # centrar las orugas sobre el cruce
    if t == 2:
        motion.turn(180)
    else:
        motion.turn(80 * t)
    motion.spin_to_line(1 if t > 0 else -1, timeout=1.5)
    reset()


def follow_until(cond, speed=None, timeout=10.0, green=False):
    """Sigue la línea hasta que cond(bits) sea verdadero.

    green=True activa los giros por marcas verdes (nivel 1). Si la línea se pierde más de
    LOST_LINE_S segundos, devuelve False."""
    if speed is None:
        speed = cfg.SPEED_LINE
    reset()
    t0 = hal.now()
    lost_since = None
    while hal.now() - t0 < timeout:
        if hal.abort_requested():
            break
        bits = hal.line_bits()
        if cond(bits):
            hal.stop()
            return True
        if green and cfg.GREEN_TURNS:
            if _pending_turn == 0:
                _check_green()
            elif is_intersection(bits):
                _take_pending_turn()
                continue
        if sees_line(bits):
            lost_since = None
        elif lost_since is None:
            lost_since = hal.now()
        elif hal.now() - lost_since > cfg.LOST_LINE_S:
            break
        step(speed)
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    return False


# ---------------------------------------------------------------- condiciones de parada
def at_intersection(count=1):
    state = {"n": 0, "on": False}

    def cond(bits):
        x = is_intersection(bits)
        if x and not state["on"]:
            state["n"] += 1
        state["on"] = x
        return state["n"] >= count
    return cond


def at_color(name, probes=("L1", "R1")):
    def cond(bits):
        for p in probes:
            if colors.read(p) == name:
                return True
        return False
    return cond


def after_cm(cm):
    hal.reset_odometer()

    def cond(bits):
        return hal.odometer_cm() >= cm
    return cond


def obstacle_within(cm):
    def cond(bits):
        d = hal.distance_cm()
        return 0 < d <= cm
    return cond


def line_end():
    def cond(bits):
        return not sees_line(bits)
    return cond
