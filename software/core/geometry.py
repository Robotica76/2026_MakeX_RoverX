## @file geometry.py
#  @brief Cuentas que dependen de la geometría del robot (cfg.P).
#  @details Así, cambiar la geometría (otro chasis, otra garra, el sensor más adelante) solo
#  requiere medir de nuevo y ajustar cfg.P; las misiones no cambian. Esquema en docs/geometria.md.
#  Los parámetros "_fwd" se miden desde el punto de referencia (sobre la caja del mBot2); aquí se
#  convierten al CENTRO DE GIRO (punto medio de las orugas). Todo en cm.

import math
import config as cfg

## @brief Atajo para leer un parámetro ajustable.
#  @param key Clave de cfg.P.
#  @return cfg.P[key].
def p(key):
    return cfg.P[key]


## @brief Convierte una distancia medida desde la referencia al centro de giro.
#  @param key Clave de cfg.P con una distancia "_fwd" (desde el punto de referencia).
#  @return cm hacia adelante desde el CENTRO DE GIRO.
def fwd(key):
    return p(key) - p("ref_rot_fwd")


## @brief Distancia entre los centros de las dos orugas.
#  @return cm (robot_w - belt_w).
def track_w():
    return p("robot_w") - p("belt_w")


## @brief Del centro de giro a cada extremo del tramo apoyado.
#  @return cm (eje de la rueda blanca / eje de la 1a negra = track_c / 2).
def contact_half():
    return p("track_c") / 2.0


## @brief Del centro de giro al eje de la rueda de la nariz, en horizontal.
#  @return cm.
def nose_fwd():
    return contact_half() + p("track_d") * math.cos(math.radians(p("nose_deg")))


## @brief Altura del eje de la nariz sobre los ejes apoyados.
#  @return cm.
def nose_h():
    return p("track_d") * math.sin(math.radians(p("nose_deg")))


## @brief Del centro de giro a la punta trasera de las orugas.
#  @return cm.
def rear_len():
    return contact_half() + p("wheel_d") / 2.0 + p("belt_t")


## @brief Del centro de giro a la punta delantera de las orugas (la nariz).
#  @return cm.
def track_front():
    return nose_fwd() + p("wheel_d") / 2.0 + p("belt_t")


## @brief Largo de la oruga de punta a punta, visto desde arriba.
#  @return cm.
def track_len():
    return rear_len() + track_front()


## @brief Del centro de giro a lo más adelantado del robot.
#  @details Lo que esté más adelante: la nariz de las orugas o la punta de la garra en posición
#  de viaje (h_carry).
#  @return cm.
def front_len():
    claw = claw_reach(p("h_carry")) + p("claw_tip")
    track = track_front()
    return claw if claw > track else track


## @brief Largo total del robot (cola a lo más adelantado).
#  @return cm.
def robot_len():
    return rear_len() + front_len()


## @brief Avance de la oruga por grado de encoder.
#  @details Usa el diámetro efectivo sprocket_d y el factor de patinaje slip.
#  @return cm por grado (promedio de las dos orugas).
def cm_per_deg():
    return math.pi * p("sprocket_d") / 360.0 * p("slip")


## @brief cm por grado de cada oruga.
#  @details Las orugas no avanzan igual por grado (prueba de 1 m: 63.5 y 65 cm, lr_ratio);
#  su promedio es cm_per_deg().
#  @return (izquierda, derecha) en cm por grado.
def cm_per_deg_lr():
    k = cm_per_deg()
    r = p("lr_ratio")
    return k * 2.0 * r / (1.0 + r), k * 2.0 / (1.0 + r)


# ---------------------------------------------------------------- brazo (tabla medida)
ARM_ANGLES = (0, 45, 90)     # ángulos del servo de la tabla (fotos); fuera de 0..90 no se usa


## @brief Tabla medida del brazo.
#  @return ((ángulo, altura, adelanto), ...) para el servo a 0, 45 y 90 grados.
def _arm_rows():
    return ((0, p("arm_h0"), p("arm_x0")), (45, p("arm_h45"), p("arm_x45")),
            (90, p("arm_h90"), p("arm_x90")))


## @brief Posición de la garra para un ángulo del servo del brazo.
#  @details Interpola la tabla medida (el brazo no es un paralelogramo: la garra gira al subir).
#  @param servo Ángulo del servo del brazo en grados.
#  @return (altura del centro de la garra sobre el piso, cm delante de la nariz).
def arm_at(servo):
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


## @brief Ángulo del servo del brazo para una altura de la garra.
#  @param h Altura deseada del centro de la garra sobre el piso (cm).
#  @return Grados del servo; fuera del alcance, el tope (0 o 90).
def arm_servo(h):
    rows = _arm_rows()
    if h <= rows[0][1]:
        return rows[0][0]
    for i in range(len(rows) - 1):
        a0, h0, _ = rows[i]
        a1, h1, _ = rows[i + 1]
        if h <= h1:
            return a0 + (h - h0) / (h1 - h0) * (a1 - a0)
    return rows[-1][0]


## @brief Distancia horizontal del centro de giro al centro de la garra.
#  @param h Altura de la garra (cm).
#  @return cm.
def claw_reach(h):
    return track_front() + arm_at(arm_servo(h))[1]


# ---------------------------------------------------------------- distancias de maniobra
## @brief Avance tras detectar un cruce con las sondas para girar SOBRE él.
#  @return cm (las sondas van sensor_fwd delante del centro de giro).
def turn_advance():
    return fwd("sensor_fwd")


## @brief Avance para que la garra llegue a un objeto.
#  @param target cm del centro de giro al centro del objeto, en la dirección en la que mira el robot.
#  @param h Altura de la garra (cm).
#  @return cm a avanzar (negativo = retroceder).
def reach_advance(target, h):
    return target - claw_reach(h)


## @brief Avance para dejar la garra sobre el centro de un pedestal (sin ultrasónico).
#  @details El pedestal está a from_cross cm de un cruce; cuando las sondas detectaron el cruce,
#  el centro de giro estaba sensor_fwd atrás.
#  @param h Altura de la garra (cm).
#  @param from_cross cm del cruce al centro del pedestal (medida de la pista, p. ej.
#  pista.nivel_1_cruce_a_pedestal_cm).
#  @param traveled cm que ya avanzó el robot desde que las sondas vieron el cruce.
#  @return cm a avanzar.
def pedestal_advance(h, from_cross, traveled):
    return from_cross - claw_reach(h) + fwd("sensor_fwd") - traveled


## @brief Avance siguiendo la línea hasta quedar a side_margin de la torre.
#  @param traveled cm que ya avanzó desde que las sondas salieron de la zona verde.
#  @return cm a avanzar.
def tower_advance(traveled):
    return p("tower_from_edge") - front_len() - p("side_margin") + fwd("sensor_fwd") - traveled


## @brief Avance después de quedar plano arriba de una rampa para estar completamente dentro.
#  @return cm.
def ramp_extra():
    return rear_len() + p("ramp_margin")


## @brief Avance desde que las sondas entran a una zona hasta que todo el robot está dentro.
#  @return cm.
def zone_extra():
    return fwd("sensor_fwd") + rear_len() + p("ramp_margin")


## @brief Desplazamiento lateral para rodear la torre sin tocarla.
#  @return cm.
def detour_out():
    return p("robot_w") / 2.0 + p("tower_size") / 2.0 + p("side_margin")


## @brief Avance paralelo para rebasar la torre por completo.
#  @details Partiendo de side_margin antes de ella.
#  @return cm.
def detour_pass():
    return front_len() + p("tower_size") + rear_len() + 2 * p("side_margin")


## @brief Cuánto moverse para dejar el centro de giro frente a una tarjeta.
#  @param card_odo Odometría (cm) cuando la sonda detectó la tarjeta.
#  @param now_odo Odometría actual (cm).
#  @return cm a moverse (negativo = atrás).
def card_to_center(card_odo, now_odo):
    return card_odo + fwd("card_probe_fwd") - now_odo


## @brief Posición lateral de una sonda.
#  @param probe Nombre de la sonda ("L2", "L1", "R1", "R2").
#  @return cm, negativo = izquierda.
def probe_x(probe):
    i = cfg.PROBES.index(probe)
    return probe_lat(i, p("probe_pitch"), p("probe_outer"))


## @brief Posición lateral de la sonda i (función pura).
#  @details Las sondas no están igual de separadas: las dos centrales a pitch entre sí y cada
#  exterior a outer de su central.
#  @param i Índice 0..3, de izquierda a derecha.
#  @param pitch cm entre las dos sondas centrales.
#  @param outer cm de cada central a la exterior.
#  @return cm laterales.
def probe_lat(i, pitch, outer):
    inner = pitch / 2.0
    return (-inner - outer, -inner, inner, inner + outer)[i]
