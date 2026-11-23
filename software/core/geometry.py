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


def contact_half():
    """Del centro de giro a cada extremo del tramo apoyado (eje blanco / eje 1a negra)."""
    return p("track_c") / 2.0


def nose_fwd():
    """Del centro de giro al eje de la rueda de la nariz (horizontal)."""
    return contact_half() + p("track_d") * math.cos(math.radians(p("nose_deg")))


def nose_h():
    """Altura del eje de la nariz sobre los ejes apoyados (cm)."""
    return p("track_d") * math.sin(math.radians(p("nose_deg")))


def rear_len():
    """Del centro de giro a la parte trasera (punta trasera de las orugas)."""
    return contact_half() + p("wheel_d") / 2.0 + p("belt_t")


def track_front():
    """Del centro de giro a la punta delantera de las orugas (la nariz)."""
    return nose_fwd() + p("wheel_d") / 2.0 + p("belt_t")


def track_len():
    """Largo de la oruga de punta a punta, visto desde arriba."""
    return rear_len() + track_front()


def front_len():
    """Del centro de giro a lo más adelantado del robot (oruga o garra en posición de viaje)."""
    claw = claw_reach(p("h_carry")) + p("claw_tip")
    track = track_front()
    return claw if claw > track else track


def robot_len():
    return rear_len() + front_len()


def cm_per_deg():
    """cm que avanza la oruga por grado de encoder."""
    return math.pi * p("sprocket_d") / 360.0 * p("slip")


def cm_per_deg_lr():
    """(izquierda, derecha): cm por grado de cada oruga. No avanzan igual por grado (prueba de
    1 m: 63.5 y 65); su promedio es cm_per_deg()."""
    k = cm_per_deg()
    r = p("lr_ratio")
    return k * 2.0 * r / (1.0 + r), k * 2.0 / (1.0 + r)


# ---------------------------------------------------------------- brazo (tabla medida)
ARM_ANGLES = (0, 45, 90)     # ángulos del servo de la tabla (fotos); fuera de 0..90 no se usa


def _arm_rows():
    return ((0, p("arm_h0"), p("arm_x0")), (45, p("arm_h45"), p("arm_x45")),
            (90, p("arm_h90"), p("arm_x90")))


def arm_at(servo):
    """(altura de la garra sobre el piso, cm delante de la nariz) con el servo a ese ángulo."""
    rows = _arm_rows()
    if servo <= rows[0][0]:
        return rows[0][1], rows[0][2]
    for i in range(len(rows) - 1):
        a0, h0, x0 = rows[i]
        a1, h1, x1 = rows[i + 1]
        if servo <= a1:
            f = (servo - a0) / float(a1 - a0)
            return h0 + f * (h1 - h0), x0 + f * (x1 - x0)
    return rows[-1][1], rows[-1][2]


def arm_servo(h):
    """Ángulo del servo del brazo para la altura h (cm sobre el piso); fuera del alcance, el tope."""
    rows = _arm_rows()
    if h <= rows[0][1]:
        return rows[0][0]
    for i in range(len(rows) - 1):
        a0, h0, _ = rows[i]
        a1, h1, _ = rows[i + 1]
        if h <= h1:
            return a0 + (h - h0) / (h1 - h0) * (a1 - a0)
    return rows[-1][0]


def claw_reach(h):
    """Distancia horizontal del centro de giro al centro de la garra, con la garra a la altura h."""
    return track_front() + arm_at(arm_servo(h))[1]


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
    return probe_lat(i, p("probe_pitch"), p("probe_outer"))


def probe_lat(i, pitch, outer):
    """Sonda i (0..3, de izquierda a derecha) -> cm laterales (no están igual de separadas)."""
    inner = pitch / 2.0
    return (-inner - outer, -inner, inner, inner + outer)[i]
