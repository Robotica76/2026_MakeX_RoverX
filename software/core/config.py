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
    # --- geometría del robot (MEDIR). Centro de giro = punto medio entre las orugas.
    "track_w": 13.0,         # distancia entre centros de las orugas
    "robot_len": 30.0,       # largo total con la garra en posición de viaje
    "robot_w": 18.0,         # ancho total
    "rear_len": 9.0,         # del centro de giro a la parte trasera
    "sensor_fwd": 7.0,       # del centro de giro a las sondas del Quad RGB
    "probe_pitch": 1.3,      # separación entre sondas vecinas del Quad RGB (MEDIR)
    "claw_fwd": 20.0,        # del centro de giro al punto de agarre de la garra
    "card_probe_fwd": 7.0,   # del centro de giro a la sonda que lee tarjetas
    "sprocket_d": 4.4,       # diámetro efectivo de la rueda motriz de la oruga
    "slip": 1.0,             # factor de patinaje (distancia real / teórica)

    # --- brazo de paralelogramo (MEDIR). Altura = altura del pivote + largo * sen(ángulo)
    "arm_pivot_h": 9.0,      # altura del eje del servo del brazo sobre el piso
    "arm_len": 8.0,          # largo de las barras del paralelogramo
    "arm_grip_dz": -2.0,     # del extremo del brazo al centro de la garra (vertical)
    "arm_zero": 90,          # ángulo del servo con el brazo HORIZONTAL (CALIBRAR)
    "arm_dir": -1,           # +1 si subir el ángulo sube el brazo, -1 si lo baja
    "h_floor": 2.5,          # altura de agarre: cubo/pelota en el piso
    "h_pedestal": 12.5,      # sobre el pedestal (10 cm + medio cubo)
    "h_carry": 17.0,         # transporte (>= 5 cm sobre la base)
    "h_bar": 12.0,           # barra de pelotas (10 cm)

    # --- garra (CALIBRAR)
    "claw_open": 90,
    "claw_cube": 35,
    "claw_ball": 45,

    # --- línea
    "line_mode": 1,          # 1 = analógico calibrado (recomendado), 0 = get_offset_track
    "dark_th": 0.45,         # 0..1 desde qué oscuridad una sonda "ve" línea
    "cross_win": 4.0,        # cm de avance en los que cuentan las dos sondas exteriores
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
    "ped1_from_cross": 12.0, # nivel 1: del cruce frente al cubo al centro del pedestal
    "ped2_from_cross": 12.0, # nivel 2: del cruce antes del pedestal al centro del pedestal
    "ball_ped_dist": 15.0,   # nivel 3: del centro de la línea (en la marca roja) al centro del pedestal
    "tower_from_edge": 25.0, # nivel 4: del borde de salida de la zona verde a la cara de la torre
    "tower_size": 10.0,
    "side_margin": 4.0,      # holgura lateral al rodear
    "bar_side": -1,          # -1 = barra de pelotas a la izquierda de la línea, 1 = derecha
    "bar_reach": 10.0,       # avance lateral hasta la barra

    # --- colores
    "color_mode": 0,         # 0 = nombre del sensor (get_color_sta), 1 = RGB más cercano
}

# Menú de ajustes: (grupo, clave, paso, mínimo, máximo). El orden es el del menú.
TUNE = [
    ("Linea", "kp", 0.05, 0, 3), ("Linea", "kd", 0.1, 0, 10), ("Linea", "v_line", 5, 10, 100),
    ("Linea", "dark_th", 0.05, 0.1, 0.9), ("Linea", "cross_win", 0.5, 1, 15),
    ("Linea", "line_mode", 1, 0, 1), ("Linea", "green", 1, 0, 1),
    ("Geometria", "sensor_fwd", 0.5, 0, 30), ("Geometria", "rear_len", 0.5, 0, 30),
    ("Geometria", "claw_fwd", 0.5, 0, 40), ("Geometria", "card_probe_fwd", 0.5, 0, 30),
    ("Geometria", "track_w", 0.5, 5, 30), ("Geometria", "robot_len", 0.5, 10, 50),
    ("Geometria", "robot_w", 0.5, 10, 40), ("Geometria", "sprocket_d", 0.1, 2, 10),
    ("Geometria", "slip", 0.02, 0.5, 1.5),
    ("Brazo", "arm_zero", 2, 0, 180), ("Brazo", "arm_dir", 2, -1, 1),
    ("Brazo", "arm_pivot_h", 0.5, 0, 30), ("Brazo", "arm_len", 0.5, 2, 30),
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
