# Rover X 2026: constantes del robot (mBot2 + kit Rover).
#
# Todo lo que se ajusta en pista está aquí. Los valores marcados "CALIBRAR" son
# estimaciones iniciales: se miden con tools/robot/api_check.py y en la pista de prueba.

# ---------------------------------------------------------------- puertos
LINE_SENSOR = 1          # índice del Quad RGB (mbuild)
US_SENSOR = 1            # índice del Ultrasónico 2
SERVO_CLAW = "s1"        # garra (abrir / cerrar)
SERVO_LIFT = "s2"        # brazo (subir / bajar)

# ---------------------------------------------------------------- signos (verificar con api_check)
LEFT_SIGN = 1            # EM1 = izquierda; avanzar = drive_power(+, -)
RIGHT_SIGN = -1          # EM2 = derecha, invertido
HEADING_SIGN = -1        # CALIBRAR: girar a la DERECHA debe dar rumbo POSITIVO
PITCH_SIGN = 1           # CALIBRAR: nariz ARRIBA debe dar pitch POSITIVO
LINE_SIGN = 1            # CALIBRAR: línea a la izquierda del sensor -> offset negativo
PAD_Y_SIGN = 1           # Ly positivo = adelante (ejemplo oficial)

# ---------------------------------------------------------------- velocidades (% de potencia)
SPEED_LINE = 45
SPEED_LINE_SLOW = 25
SPEED_STRAIGHT = 45
SPEED_TURN_MAX = 45
SPEED_TURN_MIN = 18      # por debajo las orugas no giran
SPEED_RAMP = 70
SPEED_STAIRS = 100
SPEED_PUSH = 30          # empujando el prisma

# ---------------------------------------------------------------- seguidor de línea (PID sobre offset -100..100)
LINE_KP = 0.45
LINE_KI = 0.0
LINE_KD = 1.5
LINE_I_MAX = 200
LOOP_DT = 0.01           # s entre lecturas
LOST_LINE_S = 0.6        # s sin línea antes de considerarla perdida

# Bits de get_line_sta("all"): CALIBRAR con api_check (poner cada sonda sobre negro)
LINE_BIT = {"L2": 8, "L1": 4, "R1": 2, "R2": 1}

# ---------------------------------------------------------------- giros con giroscopio
TURN_KP = 1.0
TURN_TOL_DEG = 2.5
TURN_SETTLE_S = 0.15
TURN_TIMEOUT_S = 3.0
HEADING_KP = 1.2         # mantener rumbo al ir recto

# ---------------------------------------------------------------- odometría
CM_PER_DEG = 0.0175      # CALIBRAR: avanzar 100 cm con mbot2.straight y leer grados del encoder

# ---------------------------------------------------------------- rampas y escalera
PITCH_ON_RAMP = 10       # grados: ya va subiendo
PITCH_FLAT = 4           # grados: ya está plano arriba
FLAT_CONFIRM_S = 0.4
RAMP_EXTRA_CM = 12       # avanzar después de llegar plano (quedar "completamente dentro")
RAMP_TIMEOUT_S = 6.0
STAIRS_TIMEOUT_S = 8.0

# ---------------------------------------------------------------- garra
CLAW_OPEN = 90           # CALIBRAR
CLAW_CLOSED = 30
CLAW_BALL = 40           # cerrado para la pelota de 3 cm
LIFT_DOWN = 170
LIFT_PEDESTAL = 120      # altura del pedestal (10 cm)
LIFT_UP = 60             # cubo / pelota >= 5 cm sobre la base
LIFT_BAR = 100           # altura de la barra de pelotas (10 cm)
SERVO_STEP_S = 0.35      # tiempo para que el servo llegue
HOLD_VALID_S = 5.5       # reglamento: sostener >= 5 s

# ---------------------------------------------------------------- pedestal / torre (ultrasónico, cm)
PEDESTAL_APPROACH_CM = 6
TOWER_DETECT_CM = 12
TOWER_SIDE = "left"      # lado por donde se rodea la torre
TOWER_OUT_CM = 18        # separación lateral
TOWER_PASS_CM = 28       # largo para rebasarla por completo

# ---------------------------------------------------------------- colores
# "builtin" usa get_color_sta del sensor; "nearest" usa la tabla de abajo (RGB calibrado).
COLOR_MODE = "builtin"
COLOR_REF = {            # CALIBRAR con api_check sobre cada tarjeta / zona
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
CARD_PROBE = "R2"        # sonda que pasa sobre las tarjetas (CALIBRAR según pista)
CARD_COUNT = 6
CARD_SCAN_MAX_CM = 120
BAR_SIDE = "left"        # lado de la barra de pelotas respecto a la línea (nivel 4)
BAR_APPROACH_CM = 10     # avance lateral hasta la barra
BALL_OFFSET_CM = 0       # CALIBRAR: distancia entre la sonda de tarjetas y la garra
GOAL_EXTRA_CM = 10       # avanzar tras detectar el azul para quedar completamente dentro

# Zonas de inicio por nivel (reglamento: reinicio en la zona de color del nivel alcanzado)
ZONE_LEVEL = {"red": 1, "yellow": 2, "blue": 3, "green": 4}
GOAL_COLOR = "blue"      # meta (X azul) en el nivel 4

# Marcas verdes de giro en intersecciones (nivel 1)
GREEN_TURNS = True

# ---------------------------------------------------------------- tiempos de partido
MATCH_S = 300            # 5 minutos de reloj (reglamento 1.)
AUTO_BUDGET_S = 300      # PREGUNTA ABIERTA: ¿el autónomo comparte los 5 min con el manual?

# ---------------------------------------------------------------- control manual
PAD_DEADZONE = 8
PAD_EXPO = 0.5           # 0 = lineal, 1 = cúbico
PAD_FAST = 100
PAD_SLOW = 45
PAD_TURN_GAIN = 0.8
