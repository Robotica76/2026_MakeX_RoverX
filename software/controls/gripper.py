# Rover X 2026: garra (servo de cierre + servo de brazo).

import hal
import config as cfg


def _move(port, angle):
    hal.servo(port, angle)
    hal.wait(cfg.SERVO_STEP_S)


def open_claw():
    _move(cfg.SERVO_CLAW, cfg.CLAW_OPEN)


def close_claw(angle=None):
    _move(cfg.SERVO_CLAW, cfg.CLAW_CLOSED if angle is None else angle)


def lift(angle):
    _move(cfg.SERVO_LIFT, angle)


def home():
    """Posición de viaje: brazo arriba, garra abierta."""
    lift(cfg.LIFT_UP)
    open_claw()


def hold(seconds=None):
    """Mantiene el objeto quieto el tiempo que pide el reglamento (>= 5 s)."""
    if seconds is None:
        seconds = cfg.HOLD_VALID_S
    hal.stop()
    t0 = hal.now()
    while hal.now() - t0 < seconds:
        if hal.abort_requested():
            return False
        hal.show("Sosteniendo %d" % int(seconds - (hal.now() - t0) + 1), line=2)
        hal.wait(0.1)
    return True
