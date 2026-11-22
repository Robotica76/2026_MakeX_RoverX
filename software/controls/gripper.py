# Rover X 2026: garra (servo de cierre) + brazo de paralelogramo (servo de altura).
#
# El brazo se manda por ALTURA en cm (geometry.arm_servo convierte a ángulo del servo), así
# que al cambiar el brazo solo se miden de nuevo arm_pivot_h, arm_len y arm_zero.

import hal
import config as cfg
import geometry as geo


def _move(port, angle):
    hal.servo(port, angle)
    hal.wait(cfg.SERVO_STEP_S)


def claw(angle):
    _move(cfg.SERVO_CLAW, angle)


def open_claw():
    claw(cfg.P["claw_open"])


def arm_to(h):
    """Garra a la altura h (cm sobre el piso)."""
    _move(cfg.SERVO_ARM, geo.arm_servo(h))


def home():
    """Posición de viaje: brazo arriba, garra abierta."""
    arm_to(cfg.P["h_carry"])
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
