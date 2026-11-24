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
    # oruga de 3 ruedas (vista lateral): rueda blanca atrás y dos negras adelante; la última
    # negra va levantada (nariz) para subir escalones. El robot se apoya entre la blanca y la
    # primera negra: ahí está el centro de giro (punto medio del apoyo). geometry.track_len()
    "track_c": 12.1,         # MEDIDO (C): eje rueda blanca -> eje 1a negra (tramo apoyado en el piso)
    "track_d": 4.85,         # MEDIDO (D): eje 1a negra -> eje 2a negra (nariz levantada)
    "nose_deg": 45.0,        # TODO(medir) inclinación de D sobre el piso (estimada de la foto)
    "wheel_d": 3.95,         # MEDIDO: diámetro de las ruedas (blanca y negras miden igual)
    "belt_t": 0.4,           # TODO(medir) grosor de la banda (de la foto)
    "belt_w": 3.0,           # MEDIDO (B, vista superior): ancho de la banda de cada oruga
    "robot_w": 21.35,        # MEDIDO: A 15.35 (hueco entre orugas) + 2*B; borde exterior a borde exterior
    # MEDIDO (vista inferior): sondas 11.25 cm delante del eje de los motores (rueda blanca) =
    # 11.25 - track_c/2 = 5.2 delante del centro de giro; aquí desde la referencia
    "sensor_fwd": 9.7,       # = ref_rot_fwd (4.5) + 5.2; si se mide ref_rot_fwd, ajustar
    "probe_pitch": 1.25,     # MEDIDO: entre las dos sondas centrales del Quad RGB (L1-R1)
    "probe_outer": 2.4,      # MEDIDO: de cada sonda central a la de afuera (L1-L2, R1-R2)
    "card_probe_fwd": 9.7,   # la misma barra del Quad RGB (ver sensor_fwd)
    # PRUEBA "avanza 100 cm" de mBlock: la oruga izquierda avanzó 63.5 y la derecha 65. mBlock
    # cuenta con la rueda del mBot2 (6.5 cm, TODO confirmar): diámetro efectivo de la oruga =
    # 6.5 * 64.25/100 = 4.18 (rueda 3.95 + la banda, cuadra). Verificar con "Probar recta 50".
    "sprocket_d": 4.18,      # CALIBRADO: diámetro efectivo (promedio de las dos orugas)
    # El programa de prueba del equipo (FuncionesEnPython.mblock) para los giros cuando
    # lectura * 1.3 >= ángulo pedido "por error del sensor". Puede ser escala del giroscopio
    # (entonces gyro_scale = 1.3) o inercia al parar con giro sin frenado (aquí turn_to ya frena
    # cerca del objetivo). TODO(medir): girar a mano 360° y leer el giroscopio.
    "gyro_scale": 1.0,       # TODO(calibrar) grados reales por grado leído
    "lr_ratio": 0.977,       # CALIBRADO: cm por grado izquierda / derecha (63.5 / 65)
    "skid": 1.3,             # TODO(calibrar) derrape al girar: B efectivo = distancia entre orugas * skid
                             # (girar 360° en el lugar: grados de encoder vs giroscopio)
    "slip_tol": 0.05,
    "track_max": 15.0,       # grados máximos de corrección al volver a la recta (con 30 la esquina
                             # delantera tocaba el muro del pasillo al salir)
    "track_k": 4.0,          # grados de corrección de rumbo por cm de desvío lateral (odometry.track)        # grados por ciclo de diferencia giroscopio/encoders para sospechar patinaje
    "slip": 1.0,             # TODO(calibrar con "Probar recta 50") distancia real / teórica

    # --- brazo (fotos con el servo a 0, 45 y 90 grados). NO es un paralelogramo: la garra gira
    # al subir (a 90° apunta hacia arriba, encima de la nariz). Se describe con una TABLA: para
    # cada ángulo del servo, altura del centro de la garra (entre los dedos) sobre el piso y
    # cuánto va DELANTE de la nariz de las orugas (+ adelante). Entre ángulos se interpola.
    # TODO(medir): las alturas y distancias son estimadas de las fotos; medir con regla.
    "arm_h0": 8.5,           # TODO(medir) servo 0°: brazo al frente, ~10° bajo la horizontal
    "arm_x0": 8.0,           # TODO(medir)
    "arm_h45": 14.5,         # TODO(medir) servo 45°: garra un poco inclinada hacia arriba
    "arm_x45": 6.0,          # TODO(medir)
    "arm_h90": 23.0,         # TODO(medir) servo 90°: garra apuntando hacia arriba, sobre la nariz
    "arm_x90": 0.0,          # TODO(medir)
    "arm_pivot_fwd": 10.0,   # TODO(medir) referencia -> eje fijo de arriba del brazo (solo para dibujar)
    "arm_pivot_h": 12.2,     # MEDIDO (G): piso -> eje fijo de arriba del brazo
    # Cuadrilátero articulado del brazo (solo para dibujarlo; la física usa la tabla de arriba).
    # Mástil fijo: eje de arriba (arm_pivot) y, debajo, el eje del servo. El servo gira la barra
    # de abajo (manivela); la barra de arriba (balancín) gira en el eje fijo; la barra que une sus
    # puntas (acoplador) lleva la garra atornillada, por eso la garra se inclina al subir.
    # TODO(medir): estimado de las fotos (escala por las ruedas de 3.95 cm).
    "arm_ground": 5.0,       # TODO(medir) eje fijo de arriba -> eje del servo (hacia abajo)
    "arm_ground_dx": 0.7,    # TODO(medir) el eje del servo va así de adelante del eje fijo
    "arm_crank": 7.2,        # TODO(medir) barra de abajo: eje del servo -> articulación del acoplador
    "arm_rocker": 5.75,      # TODO(medir) barra de arriba: eje fijo -> articulación del acoplador
    "arm_coupler": 7.0,      # TODO(medir) acoplador: articulación de abajo -> de arriba
    "arm_claw_u": 1.9,       # TODO(medir) centro de la garra: cm sobre la articulación de abajo
    "arm_claw_v": 7.9,       # TODO(medir) centro de la garra: cm delante del acoplador
    "claw_tip": 3.0,         # TODO(medir) del centro de la garra a la punta de los dedos
    "h_floor": 8.5,          # lo más bajo que llega (servo 0°); el brazo no alcanza el piso
    "h_pedestal": 12.5,      # sobre el pedestal (10 cm + medio cubo)
    "h_carry": 17.5,         # transporte: base del cubo >= 5 cm sobre el pedestal (10 + 5 + 2.5)
    "h_bar": 12.0,           # barra de pelotas (10 cm)
    "h_prism": 8.5,          # lo más baja (servo 0°), abierta: lleva el prisma como en una cuna (misión 7)

    # --- garra (CALIBRAR)
    "claw_open": 90,         # TODO(calibrar)
    "claw_cube": 35,         # TODO(calibrar)
    "claw_ball": 45,         # TODO(calibrar)
    # centro de masa (lo usa el simulador para la volcadura; TODO medir: equilibrar el robot
    # sobre un lápiz de frente y de lado)
    "com_fwd": -1.0,         # TODO(medir) centro de masa del chasis, adelante del centro de giro
    "com_h": 6.5,            # TODO(medir) altura del centro de masa del chasis sobre el piso
    "arm_mass": 0.15,        # TODO(medir) fracción del peso que está en brazo + garra
    "arm_w": 4.9,            # MEDIDO (F, vista superior): ancho del marco del brazo
    "arm_lat": -0.75,        # MEDIDO (E = 5.97 de la oruga izquierda al marco): centro del marco
                             # 0.75 a la DERECHA del centro (+ = izquierda); (15.35-4.9)/2 - 5.97
    "claw_open_w": 9.0,      # TODO(medir) separación interior de los dedos con la garra abierta
                             # (el prisma mide ~7 cm de base en el plano; pregunta abierta)

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
    # control contra volcadura (activo en rampas y escalera)
    "tip_pitch": 8,          # grados: desde aquí la potencia solo sube de a poco
    "tip_slew": 120,         # %/s: cuánto puede subir la potencia por segundo inclinado
    "slope_min_v": 4.0,      # cm/s: subiendo más lento que esto (o resbalando) -> asistencia
    "slope_boost": 60,       # %/s: cuánto sube la potencia extra mientras haga falta (máx. +60)
    "h_top": 23.0,           # servo a 90°: garra arriba, encima de la nariz (rodear la torre)
    "h_climb": 10.5,         # garra baja y al frente al subir (adelanta y baja el centro de masa).
                             # No a 8.5 (servo 0°): el brazo baja al detectar la rampa (~10° de
                             # pitch, cola aún en el piso) y el cubo pasaba a 1.8 cm de la rampa
                             # (0.4 cm con una rampa 20 % más empinada). Con 10.5: 4.0 / 2.8 cm.

    # --- medidas de la PISTA (MEDIR en la pista de prueba; sin ultrasónico son la referencia)
    # TODO(medir en pista): valores sacados del plano del reglamento (simulation/arena.py)
    "ped1_from_cross": 12.4, # TODO(pista) nivel 1: de donde las sondas VEN el cruce (su orilla cercana) al centro del pedestal
    "ped2_from_cross": 11.1, # nivel 2: del cruce antes del pedestal al centro del pedestal
    "ball_ped_dist": 23.6,   # nivel 3: del centro de la línea (en la marca roja) al centro del pedestal
    "tower_from_edge": 7.5,  # nivel 4: del borde de salida de la zona verde a la cara de la torre
    "tower_size": 10.0,
    "side_margin": 2.0,      # holgura al rodear (con 4 cm el robot volvía sobre una tarjeta)
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
    ("Geometria", "card_probe_fwd", 0.5, -10, 40), ("Geometria", "track_c", 0.5, 5, 40),
    ("Geometria", "belt_w", 0.5, 1, 10), ("Geometria", "robot_w", 0.5, 10, 40),
    ("Geometria", "sprocket_d", 0.02, 2, 10), ("Geometria", "lr_ratio", 0.005, 0.9, 1.1), ("Geometria", "gyro_scale", 0.01, 0.5, 2),
    ("Geometria", "slip", 0.02, 0.5, 1.5),
    ("Brazo", "arm_h0", 0.5, 0, 40), ("Brazo", "arm_x0", 0.5, -20, 30),
    ("Brazo", "arm_h45", 0.5, 0, 40), ("Brazo", "arm_x45", 0.5, -20, 30),
    ("Brazo", "arm_h90", 0.5, 0, 40), ("Brazo", "arm_x90", 0.5, -20, 30),
    ("Brazo", "h_pedestal", 0.5, 0, 30), ("Brazo", "h_carry", 0.5, 0, 30),
    ("Brazo", "h_bar", 0.5, 0, 30), ("Brazo", "h_prism", 0.5, 0, 30), ("Brazo", "h_floor", 0.5, 0, 30),
    ("Garra", "claw_open", 2, 0, 180), ("Garra", "claw_cube", 2, 0, 180),
    ("Garra", "claw_ball", 2, 0, 180),
    ("Movimiento", "v_slow", 5, 10, 100), ("Movimiento", "v_ramp", 5, 10, 100),
    ("Movimiento", "v_stairs", 5, 10, 100), ("Movimiento", "v_turn_min", 1, 5, 60),
    ("Movimiento", "turn_kp", 0.1, 0.1, 5),
    ("Movimiento", "tip_slew", 10, 20, 400), ("Brazo", "h_climb", 0.5, 0, 30), ("Movimiento", "head_kp", 0.1, 0, 5),
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
CARD_PROBE = "R1"        # sonda que lee las tarjetas: una CENTRAL (0.6 cm del centro). Las tarjetas miden
                         # 7 cm a lo ancho de la línea (±3.5); R2 está a 3.0 cm (medido) y con 1 cm de
                         # desvío ya leía fuera. TODO(pista): ¿tarjetas sobre la línea o al lado?
CARD_COUNT = 6
CARD_SCAN_MAX_CM = 120
ZONE_LEVEL = {"red": 1, "yellow": 2, "blue": 3, "green": 4}
GOAL_COLOR = "blue"
GOAL_RUN_CM = 9.0        # azul seguido más largo que una tarjeta = la X de la meta (tarjeta 4.7 cm; con
                         # 6 una tarjeta leída en diagonal pasaba por meta; la meta mide ~25 cm)

# ---------------------------------------------------------------- tiempos
TURN_SETTLE_S = 0.15
TURN_TIMEOUT_S = 3.0
FLAT_CONFIRM_S = 0.4
CREST_DROP = 8           # grados que baja el pitch desde su máximo para dar por pasada la cima
RAMP_TIMEOUT_S = 6.0
STAIRS_TIMEOUT_S = 8.0
SERVO_STEP_S = 0.35
HOLD_VALID_S = 5.5       # reglamento: sostener >= 5 s
MATCH_S = 300
AUTO_BUDGET_S = 300      # PREGUNTA ABIERTA: ¿el autónomo comparte los 5 min con el manual?
# Programa por TRAMOS (como lo usa el equipo): en la pantalla de inicio el joystick del CyberPi
# elige el nivel y el botón de arranque corre el tramo de ese nivel; el robot se detiene al
# terminarlo (el equipo pide reinicio, lo coloca en la zona del siguiente nivel, lo elige y arranca).
SEGMENTS = True
LEVEL_KEYS = {"up": 1, "down": 2, "left": 3, "right": 4}   # confirmado por el equipo
START_BUTTON = "b"       # empezar (autónomo: el tramo; manual: el control)
ABORT_BUTTON = "a"       # parar (autónomo y manual)
MENU_BUTTON = "middle"   # en la pantalla de inicio: centro del joystick = ajustes

# ---------------------------------------------------------------- control manual
PAD_DEADZONE = 8
PAD_EXPO = 0.5
PAD_FAST = 100
PAD_SLOW = 45
PAD_TURN_GAIN = 0.8
