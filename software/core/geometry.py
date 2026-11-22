# Rover X 2026: cuentas que dependen de la geometría del robot (cfg.P).
#
# Así, cambiar la geometría (otro chasis, otra garra, el sensor más adelante) solo requiere
# medir de nuevo y ajustar cfg.P; las misiones no cambian. Esquema en docs/geometria.md.
# Los parámetros "_fwd" se miden desde el punto de referencia (sobre la caja del mBot2); aquí
# se convierten al CENTRO DE GIRO (punto medio de las orugas). Todo en cm.

import math
import config as cfg

def p(key):
    return cfg.P[key]


def fwd(key):
    """Distancia hacia adelante desde el CENTRO DE GIRO de un punto medido desde la referencia."""
    return p(key) - p("ref_rot_fwd")


def track_w():
    """Distancia entre los centros de las dos orugas."""
    return p("robot_w") - p("belt_w")


def rear_len():
    """Del centro de giro a la parte trasera (punta trasera de las orugas)."""
    return p("track_len") / 2.0


def front_len():
    """Del centro de giro a lo más adelantado del robot (oruga o garra en posición de viaje)."""
    claw = claw_reach(p("h_carry")) + p("claw_tip")
    track = p("track_len") / 2.0
    return claw if claw > track else track


def robot_len():
    return rear_len() + front_len()


def cm_per_deg():
    """cm que avanza la oruga por grado de encoder."""
    return math.pi * p("sprocket_d") / 360.0 * p("slip")


# ---------------------------------------------------------------- brazo de paralelogramo
def arm_theta(h):
    """Ángulo del brazo (grados sobre la horizontal) para que la garra quede a la altura h."""
    tip = h - p("arm_grip_dz")
    s = (tip - p("arm_pivot_h")) / p("arm_len")
    if s > 1:
        s = 1
    if s < -1:
        s = -1
    return math.asin(s) * 180.0 / math.pi      # sin math.degrees (MicroPython)


def arm_servo(h):
    """Ángulo del servo del brazo para la altura h (cm sobre el piso)."""
    a = p("arm_zero") + p("arm_dir") * arm_theta(h)
    if a < 0:
        return 0
    if a > 180:
        return 180
    return a


def claw_reach(h):
    """Distancia horizontal del centro de giro al centro de la garra, con la garra a la altura h.
    Al subir o bajar el brazo, la garra se acerca al robot (cos del ángulo)."""
    t = arm_theta(h) * math.pi / 180.0
    return fwd("arm_pivot_fwd") + p("arm_len") * math.cos(t) + p("arm_grip_dx")


# ---------------------------------------------------------------- distancias de maniobra
def turn_advance():
    """Lo que hay que avanzar tras detectar un cruce con las sondas para girar SOBRE él."""
    return fwd("sensor_fwd")


def reach_advance(target, h):
    """Avance para que el punto de agarre (garra a la altura h) llegue a un objeto cuyo centro
    está a `target` cm del centro de giro, en la dirección en la que mira el robot."""
    return target - claw_reach(h)


def pedestal_advance(h, from_cross, traveled):
    """Sin ultrasónico: avance para dejar la garra sobre el centro de un pedestal que está a
    `from_cross` cm de un cruce, cuando el robot ya avanzó `traveled` cm desde que las sondas
    detectaron ese cruce (en ese momento el centro de giro estaba sensor_fwd atrás)."""
    return from_cross - claw_reach(h) + fwd("sensor_fwd") - traveled


def tower_advance(traveled):
    """Avance (siguiendo la línea) hasta quedar a side_margin de la torre, medido desde que las
    sondas salieron de la zona verde y ya se avanzaron `traveled` cm."""
    return p("tower_from_edge") - front_len() - p("side_margin") + fwd("sensor_fwd") - traveled


def ramp_extra():
    """Avance después de quedar plano arriba de una rampa para estar completamente dentro."""
    return rear_len() + p("ramp_margin")


def zone_extra():
    """Avance desde que las sondas entran a una zona hasta que todo el robot está dentro."""
    return fwd("sensor_fwd") + rear_len() + p("ramp_margin")


def detour_out():
    """Desplazamiento lateral para rodear la torre sin tocarla."""
    return p("robot_w") / 2.0 + p("tower_size") / 2.0 + p("side_margin")


def detour_pass():
    """Avance paralelo para rebasar la torre por completo, partiendo de side_margin antes de ella."""
    return front_len() + p("tower_size") + rear_len() + 2 * p("side_margin")


def card_to_center(card_odo, now_odo):
    """Cuánto moverse (cm, negativo = atrás) para dejar el centro de giro frente a una tarjeta
    detectada cuando la odometría marcaba card_odo."""
    return card_odo + fwd("card_probe_fwd") - now_odo


def probe_x(probe):
    """Posición lateral de cada sonda (cm, negativo = izquierda)."""
    i = cfg.PROBES.index(probe)
    return (i - 1.5) * p("probe_pitch")
