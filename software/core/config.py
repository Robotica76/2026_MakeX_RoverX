# Rover X 2026: configuración del robot (mBot2 + orugas + garra de dos servos).
#
# Dos tipos de valores:
#   - Constantes fijas (puertos, nombres de sondas): variables normales en MAYÚSCULAS.
#   - Parámetros ajustables en pista: diccionario P. Se cambian desde el menú del CyberPi
#     (software/core/settings.py), se guardan en el robot y se leen como cfg.P["clave"].
#
# Todas las distancias van en cm y los ángulos en grados. Lo marcado MEDIR sale del robot
# real (ver docs/geometria.md); lo marcado CALIBRAR sale de tools/robot/api_check.py.

# ---------------------------------------------------------------- puertos (fijos)
LINE_SENSOR = 1          # índice del Quad RGB (mbuild)
HAS_US = False           # el diseño actual NO lleva ultrasónico (ver docs/geometria.md)
US_SENSOR = 1            # índice del Ultrasónico 2, solo si HAS_US
SERVO_CLAW = "s1"        # garra (abrir / cerrar)
SERVO_ARM = "s2"         # brazo de paralelogramo (subir / bajar)
PROBES = ("L2", "L1", "R1", "R2")
PARAMS_FILE = "rover_params.json"

# ---------------------------------------------------------------- signos (CALIBRAR con api_check)
LEFT_SIGN = 1            # EM1 = izquierda; avanzar = drive_power(+, -)
RIGHT_SIGN = -1          # EM2 = derecha, invertido
HEADING_SIGN = -1        # girar a la DERECHA debe dar rumbo POSITIVO
PITCH_SIGN = 1           # nariz ARRIBA debe dar pitch POSITIVO
LINE_SIGN = 1            # (modo "builtin") línea a la izquierda -> offset negativo
PAD_Y_SIGN = 1           # Ly positivo = adelante (ejemplo oficial)
LINE_BIT = {"L2": 8, "L1": 4, "R1": 2, "R2": 1}   # bits de get_line_sta("all")

LOOP_DT = 0.01           # s entre lecturas del control

# ---------------------------------------------------------------- parámetros ajustables
P = {
    # --- geometría del robot (MEDIR, hoja en docs/medidas_robot.md).
    # TODO(medidas): los valores marcados TODO son ESTIMADOS a ojo de la foto del diseño
    # propuesto (docs/img/medidas_propuestas.webp), no medidos. Reemplazarlos con la hoja.
    # Las distancias "_fwd" se miden hacia adelante desde el PUNTO DE REFERENCIA (punto naranja
    # sobre la caja del mBot2). El centro de giro real de un robot de orugas es el punto medio de
    # las orugas a lo largo; ref_rot_fwd dice cuánto está adelante de la referencia.
    "ref_rot_fwd": 4.5,      # TODO(medir) referencia -> punto medio de las orugas (+ = adelante)
    "track_len": 19.0,       # TODO(medir) largo de la oruga (de punta a punta)
    "belt_w": 4.0,           # TODO(medir) ancho de la banda de cada oruga
    "robot_w": 18.5,         # TODO(medir) ancho total, de borde exterior a borde exterior de las orugas
    "sensor_fwd": 7.5,       # TODO(medir) referencia -> sondas del Quad RGB
    "probe_pitch": 1.3,      # TODO(medir) separación entre sondas vecinas del Quad RGB
    "card_probe_fwd": 7.5,   # TODO(medir) referencia -> sonda que lee las tarjetas (la misma barra del Quad RGB)
    "sprocket_d": 4.4,       # TODO(medir) diámetro efectivo de la rueda motriz de la oruga
    "slip": 1.0,             # TODO(calibrar con "Probar recta 50") distancia real / teórica

    # --- brazo de paralelogramo (MEDIR). Altura = altura del pivote + largo * sen(ángulo)
    "arm_pivot_fwd": 10.0,   # TODO(medir) referencia -> eje del servo del brazo (horizontal)
    "arm_pivot_h": 10.0,     # TODO(medir) altura del eje del servo del brazo sobre el piso
    "arm_len": 9.0,          # TODO(medir) largo de las barras del paralelogramo, eje a eje
    "arm_grip_dx": 3.0,      # TODO(medir) del eje del extremo del brazo al centro de la garra (horizontal)
    "arm_grip_dz": -1.0,     # TODO(medir) del eje del extremo del brazo al centro de la garra (vertical, - = abajo)
    "claw_tip": 3.0,         # TODO(medir) del centro de la garra a su punta delantera
    "arm_zero": 90,          # TODO(calibrar) ángulo del servo con el brazo HORIZONTAL
    "arm_dir": -1,           # TODO(calibrar) +1 si subir el ángulo sube el brazo, -1 si lo baja
    "h_floor": 2.5,          # altura de agarre: cubo/pelota en el piso
    "h_pedestal": 12.5,      # sobre el pedestal (10 cm + medio cubo)
    "h_carry": 17.5,         # transporte: base del cubo >= 5 cm sobre el pedestal (10 + 5 + 2.5)
    "h_bar": 12.0,           # barra de pelotas (10 cm)

    # --- garra (CALIBRAR)
    "claw_open": 90,         # TODO(calibrar)
    "claw_cube": 35,         # TODO(calibrar)
    "claw_ball": 45,         # TODO(calibrar)

    # --- línea
    "line_mode": 1,          # 1 = analógico calibrado (recomendado), 0 = get_offset_track
    "dark_th": 0.45,         # 0..1 desde qué oscuridad una sonda "ve" línea
    "cross_win": 4.0,        # cm de avance en los que cuentan las dos sondas exteriores
    "cross_th": 0.8,         # 0..1 oscuridad de una exterior para contar un cruce (> verde)
    "kp": 0.45,
    "kd": 1.5,
    "ki": 0.0,
    "lost_s": 0.6,           # s sin línea antes de darla por perdida
    "green": 0,              # 1 = girar con marcas verdes (opcional; la ruta no depende de ellas)

    # --- velocidades (% potencia)
    "v_line": 45,
    "v_slow": 25,
    "v_straight": 45,
    "v_turn_max": 45,
    "v_turn_min": 18,
    "v_ramp": 70,
    "v_stairs": 100,
    "v_push": 30,

    # --- giros y rampas
    "turn_kp": 1.0,
    "turn_tol": 2.5,
    "head_kp": 1.2,
    "pitch_ramp": 10,        # grados: ya va subiendo
    "pitch_flat": 4,         # grados: ya está plano arriba
    "ramp_margin": 3.0,      # cm extra para quedar "completamente dentro"

    # --- medidas de la PISTA (MEDIR en la pista de prueba; sin ultrasónico son la referencia)
    # TODO(medir en pista): valores sacados del plano del reglamento (simulation/arena.py)
    "ped1_from_cross": 11.1, # nivel 1: del cruce frente al cubo al centro del pedestal
    "ped2_from_cross": 11.1, # nivel 2: del cruce antes del pedestal al centro del pedestal
    "ball_ped_dist": 23.6,   # nivel 3: del centro de la línea (en la marca roja) al centro del pedestal
    "tower_from_edge": 7.5,  # nivel 4: del borde de salida de la zona verde a la cara de la torre
    "tower_size": 10.0,
    "side_margin": 4.0,      # holgura lateral al rodear
    "bar_side": 1,           # -1 = barra de pelotas a la izquierda de la línea, 1 = derecha
    "bar_reach": 8.2,        # avance desde la línea para dejar la garra sobre el agujero

    # --- colores
    "color_mode": 0,         # 0 = nombre del sensor (get_color_sta), 1 = RGB más cercano
}

# Menú de ajustes: (grupo, clave, paso, mínimo, máximo). El orden es el del menú.
TUNE = [
    ("Linea", "kp", 0.05, 0, 3), ("Linea", "kd", 0.1, 0, 10), ("Linea", "v_line", 5, 10, 100),
    ("Linea", "dark_th", 0.05, 0.1, 0.9), ("Linea", "cross_win", 0.5, 1, 15),
    ("Linea", "cross_th", 0.05, 0.3, 1),
    ("Linea", "line_mode", 1, 0, 1), ("Linea", "green", 1, 0, 1),
    ("Geometria", "ref_rot_fwd", 0.5, -20, 20), ("Geometria", "sensor_fwd", 0.5, -10, 40),
    ("Geometria", "card_probe_fwd", 0.5, -10, 40), ("Geometria", "track_len", 0.5, 5, 50),
    ("Geometria", "belt_w", 0.5, 1, 10), ("Geometria", "robot_w", 0.5, 10, 40),
    ("Geometria", "sprocket_d", 0.1, 2, 10),
    ("Geometria", "slip", 0.02, 0.5, 1.5),
    ("Brazo", "arm_zero", 2, 0, 180), ("Brazo", "arm_dir", 2, -1, 1),
    ("Brazo", "arm_pivot_fwd", 0.5, -10, 40), ("Brazo", "arm_pivot_h", 0.5, 0, 30),
    ("Brazo", "arm_len", 0.5, 2, 30), ("Brazo", "arm_grip_dx", 0.5, -10, 20),
    ("Brazo", "arm_grip_dz", 0.5, -15, 15),
    ("Brazo", "h_pedestal", 0.5, 0, 30), ("Brazo", "h_carry", 0.5, 0, 30),
    ("Brazo", "h_bar", 0.5, 0, 30), ("Brazo", "h_floor", 0.5, 0, 30),
    ("Garra", "claw_open", 2, 0, 180), ("Garra", "claw_cube", 2, 0, 180),
    ("Garra", "claw_ball", 2, 0, 180),
    ("Movimiento", "v_slow", 5, 10, 100), ("Movimiento", "v_ramp", 5, 10, 100),
    ("Movimiento", "v_stairs", 5, 10, 100), ("Movimiento", "v_turn_min", 1, 5, 60),
    ("Movimiento", "turn_kp", 0.1, 0.1, 5), ("Movimiento", "head_kp", 0.1, 0, 5),
    ("Pista", "ped1_from_cross", 0.5, 0, 60), ("Pista", "ped2_from_cross", 0.5, 0, 60),
    ("Pista", "ball_ped_dist", 0.5, 0, 60), ("Pista", "tower_from_edge", 0.5, 0, 100),
    ("Pista", "side_margin", 0.5, 0, 15), ("Pista", "bar_side", 2, -1, 1),
    ("Pista", "bar_reach", 0.5, 0, 30),
]

# Calibración de la línea por sonda: luz sobre blanco y sobre negro (menú "Calibrar linea").
LINE_CAL = {"L2": [90, 10], "L1": [90, 10], "R1": [90, 10], "R2": [90, 10]}

# Referencias RGB para color_mode = 1 (CALIBRAR con api_check sobre cada color)
COLOR_REF = {
    "black": (20, 20, 20),
    "white": (230, 230, 230),
    "red": (200, 40, 40),
    "yellow": (220, 200, 50),
    "green": (40, 160, 70),
    "blue": (40, 90, 200),
    "cyan": (60, 180, 200),
    "purple": (140, 60, 170),
}
COLOR_CONFIRM = 3        # lecturas iguales seguidas para aceptar un color
CARD_PROBE = "R2"        # sonda que pasa sobre las tarjetas
CARD_COUNT = 6
CARD_SCAN_MAX_CM = 120
ZONE_LEVEL = {"red": 1, "yellow": 2, "blue": 3, "green": 4}
GOAL_COLOR = "blue"

# ---------------------------------------------------------------- tiempos
TURN_SETTLE_S = 0.15
TURN_TIMEOUT_S = 3.0
FLAT_CONFIRM_S = 0.4
RAMP_TIMEOUT_S = 6.0
STAIRS_TIMEOUT_S = 8.0
SERVO_STEP_S = 0.35
HOLD_VALID_S = 5.5       # reglamento: sostener >= 5 s
MATCH_S = 300
AUTO_BUDGET_S = 300      # PREGUNTA ABIERTA: ¿el autónomo comparte los 5 min con el manual?

# ---------------------------------------------------------------- control manual
PAD_DEADZONE = 8
PAD_EXPO = 0.5
PAD_FAST = 100
PAD_SLOW = 45
PAD_TURN_GAIN = 0.8
