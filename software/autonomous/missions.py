# Rover X 2026: misiones del área automática, escritas como listas de pasos.
#
# BORRADOR DE RUTA: se armó con el plano de la arena del reglamento, reconstruido en
# simulation/arena.py, y se prueba con el simulador (python3 simulation/run_auto.py). Los
# conteos de cruces, giros y distancias se ajustan en la pista. La lógica de cada paso está en
# autonomous.py (run_step).
#
# Los valores pueden ser números o NOMBRES de parámetros de cfg.P ("v_line", "h_pedestal",
# "claw_cube"...). Los nombres se leen al ejecutar, así que lo que se cambie en el menú del
# CyberPi se aplica sin volver a cargar el programa.
#
# Pasos:
#   ("line", cond, vel, timeout[, verde])  seguir la línea hasta cond:
#        ("junction", n, tipo)  n cruces; tipo "any" / "X" / "L" / "R"
#        ("color", nombre)  ("leave", color) salir de una zona  ("cm", d)  ("end",)
#        ("color_cm", color, d)  d cm después de ver el color (terminar una rampa sin parar en ella)
#        ("pitch",) empieza una rampa   ("obstacle", cm) solo si hay ultrasónico
#   ("jturn", grados)          girar EN el último cruce (centro de giro sobre él) y alinear
#   ("straight", cm[, vel])    recto con giroscopio (negativo = atrás)
#   ("center",)                avanzar lo que hay de las sondas al centro de giro (para girar
#                              sobre la marca o el cruce que acaban de ver las sondas)
#   ("turn", grados)           giro relativo, positivo = derecha
#   ("ref", rumbo)             "ahora miro hacia este rumbo de mapa" (0 este, 90 sur, 180 oeste)
#   ("line_ref", rumbo, cond, vel, timeout)  seguir la línea promediando el rumbo -> referencia
#   ("head", rumbo)            girar hasta ese rumbo de mapa
#   ("drive", rumbo, cm[, vel])  girar a ese rumbo y avanzar sosteniéndolo con el IMU
#   ("pose", x, y, rumbo)      posición conocida en el mapa (cm; x este, y sur) para la odometría
#   ("goto", x, y[, vel])      ir a ese punto con odometría encoders + giroscopio (patinaje)
#   ("square", rumbo, s, "x"|"y", valor)  apoyarse en un muro (por tiempo) para fijar esa coordenada
#   ("track_line", rumbo, "x"|"y", valor, max_cm)  seguir esa recta del mapa hasta ver una línea
#   ("spin", dir)              girar hasta encontrar la línea (1 der, -1 izq)
#   ("cross", vel, max_cm[, rumbo])  recto hasta que las dos exteriores ven una línea que cruza
#   ("seek"[, vel, max_cm, rumbo])  recto con giroscopio hasta que las sondas centrales vean línea
#   ("approach_ped", altura, dist)  dejar la garra (a esa altura) sobre el centro de un pedestal
#                              que está a `dist` cm del último cruce (medida de la pista)
#   ("approach_obj", altura, dist)  igual, para un objeto a `dist` cm del centro de giro
#   ("retreat",)               regresar lo mismo que avanzó el último acercamiento
#   ("to_tower",)              seguir la línea hasta quedar junto a la torre (desde la zona verde)
#
# El robot NO lleva ultrasónico: los acercamientos se hacen con el encoder, desde un cruce o
# el borde de una zona, usando medidas de la pista (cfg.P "ped1_from_cross", etc.).
#   ("climb",) / ("stairs"[, extra_cm])   subir rampa / escalera esperando a quedar plano
#   ("if_skipped", id, [pasos]) / ("if_ran", id, [pasos])  según si la misión id se saltó (ya
#                              estaba hecha antes de un reinicio) o se corrió en esta pasada
#   ("if_placed", [pasos])       pasos si el robot recién fue colocado a mano (arranque o reinicio)
#   ("if_start", nivel, [pasos])  pasos que solo se hacen si el autónomo arrancó en ese nivel
#   ("crest", cm)              subir hasta pasar la cima y avanzar cm (poco espacio arriba)
#   ("claw", apertura) / ("arm", altura)
#   ("hold",)                  sostener 5 s (validación de cubo y pelota)
#   ("detour", lado)           rodear la torre (-1 izquierda, 1 derecha)
#   ("cards",) / ("drop_red",) / ("stop_in", color)
#
# critical=True: si falla, el robot quedó fuera de ruta y el tramo se detiene (el equipo pide
# reinicio en la zona del nivel). critical=False: se sigue con la siguiente misión.
# end_level=True: termina el tramo (el robot ya está completamente dentro del siguiente nivel y,
# en el tramo 1, el cubo ya está en el pedestal): con cfg.SEGMENTS el robot se detiene ahí; el
# equipo pide reinicio en la zona del siguiente nivel, lo elige con el joystick y arranca.

# ---- nivel 1 POR COMANDOS (experiencia del equipo: con la línea delgada y las marcas verdes
# seguir la línea no funcionaba). Cada tramo es "avanza X cm sosteniendo el rumbo Y" con el IMU,
# referido a cómo se coloca el robot en la zona roja (mirando al sur, sobre la línea vertical).
# TODO(medir en pista): las distancias salen del plano (simulation/arena.py).
L1_SOUTH_CM = 33.8          # de la zona roja al cruce de la línea inferior
L1_EAST_CM = 59.0           # del cruce hasta tener las sondas sobre el cruce frente al cubo
L1_CROSS_SLACK = 6.0        # cada tramo se queda corto esto y termina buscando el cruce
L1_NORTH_CM = 18.4          # del cruce frente al cubo a la altura de la línea que sube a la rampa
L1_TO_RAMP_CM = 18.7        # de ahí al pie de la rampa gris
# Holguras del nivel 1 (simulador con el contorno medido): la línea inferior va a 12.1 cm de la
# pared del nivel 4 (44 cm de alto) y el robot mide 21.35 x 20.3. Girando en el lugar las
# esquinas barren 16 cm (nariz) y 13.6 (cola): al girar sobre el cruce la nariz se metía 3 cm en
# la pared, y junto al pedestal ningún giro de más de 10° cabe.
L1_START = (12.6, 12.9)     # TODO(pista) centro de giro al colocarlo en la zona roja (sobre la X)
L1_TURN_X = 15.4            # bajar 2.8 cm a la derecha de la línea: la cola barre 13.6 al girar
                            # al este y la orilla izquierda de la arena está en x 0
L1_CROSS_CY = 43.2          # TODO(pista) y del centro de giro cuando las sondas ven la línea
                            # inferior: girar AHÍ (la nariz barre hasta y 59.2, la pared en 61)
L1_TURN_BACK_CM = 1.5       # y retroceder esto antes: el robot se pasa ~1 cm al frenar
L1_CUBE_Y = 47.4            # recta hacia el cubo (pedestal en y 45.6..51.8): con el robot en 47.4
                            # el pivote de después pasa a ~1.5 cm de la pared
L1_PIVOT_BACK_CM = 11.0     # retroceso tras el cubo para pivotear al norte sin barrer el pedestal
L1_PIVOT_D = 11.9           # = geo.track_w() * skid / 2: el pivote sube el centro y lo corre al este

# ---- constantes de ruta (TODO: AJUSTAR EN PISTA; salen del plano y se probaron en el simulador)
L1_BACK_CM = 4.5            # nivel 1: retroceso tras el cubo para poder girar sin barrer el pedestal
L2_TO_PED_CM = 8.8          # TODO(pista) nivel 2: línea desde la T hasta la garra sobre el pedestal
                            # (con 9.6 la nariz quedaba a 0.3 cm del pedestal)
L2_T_BACK_CM = 3.0          # TODO(pista) nivel 2: retroceso en la T antes de girar al sur
L2_BACK_CM = 6.0            # nivel 2: lo mismo frente al pedestal donde se deja el cubo
# Rumbos de mapa (pasos "ref" y "head"): 0 = este, 90 = sur, 180 = oeste, -90 = norte, en la
# vista superior del reglamento. Se fijan con "ref" sobre una recta conocida.
# Puntos del mapa en cm (x este desde la orilla izquierda, y SUR desde la orilla de arriba),
# del plano reconstruido (simulation/arena.py). TODO(medir en pista).
# Entrada (simulador con el contorno medido): la línea de la zona amarilla baja en diagonal hasta
# 12.7 cm de la pared del nivel 3 y el robot inclinado a 45° llega 16 cm: la esquina tocaba la
# pared siguiendo la línea. Ahora, desde la T, por odometría: bajar hasta CORRIDOR_TURN, girar al
# este ahí (la nariz barre hasta y 59.5, pared en 61) y entrar por el centro de la abertura del
# muro izquierdo (y 30.5 .. 60.2: 4 cm por lado).
T2_PLACED = (131.9, 30.4)        # TODO(pista) centro de giro tras colocar en la T, retroceder y girar al sur
T2_FROM_M4 = (133.9, 26.8)       # TODO(pista) centro de giro al terminar la misión 4 (mirando al sur)
CORRIDOR_TURN = (134.2, 43.5)    # girar al este: y <= 61 - 16 - 1.5
CORRIDOR_ENTRY_Y = 45.0          # centro de la abertura izquierda (y 30.5 .. 60.2)
CORRIDOR_W1 = (181.0, 42.5)      # TODO(pista) donde se gira al norte para escuadrar arriba: a 17.7 cm del muro
                                 # de abajo (la nariz barre 16 cm). x: el pivote hacia atrás barre 20.6 cm
                                 # al oeste (muro izquierdo en x 152.9) y el giro al norte 16 al este (muro
                                 # derecho en 212.5): x entre 175 y 194.5. Se llega retrocediendo desde el
                                 # muro derecho; si patina se queda más al este (hasta 13 cm de margen)
CORRIDOR_EAST_X = 212.5 - 11.85  # x del centro de giro con la nariz en el muro derecho (geo.track_front())
CORRIDOR_TO_EAST_S = 9.0         # s al este a v_slow hasta apoyarse en él (~42 cm con patinaje)
CORRIDOR_TO_WALL_S = 6.0         # s al norte a v_slow: ~35 cm hasta el muro con patinaje, y empuja
CORRIDOR_WALL_Y = 0.9 + 11.85    # y del centro de giro con la nariz en el muro (geo.track_front())
# Salida (lo mostró el simulador con el robot medido): la abertura del muro derecho mide 29.6 cm
# y el robot 21.35, quedan 4 cm por lado. Un giro EN EL LUGAR con una oruga sobre un recorte
# corría el robot hasta 6 cm sin que la odometría lo viera, y un retroceso con las DOS orugas
# sobre recortes también (retrocedía 10 cm creyendo 17): chocaba con la punta del muro de arriba.
# Ahora, desde el muro mismo (y exacta), se gira HACIA ATRÁS SOBRE LA ORUGA IZQUIERDA (quieta, no
# patina): el centro gira 11.9 cm alrededor de ella y queda al este, 11.9 al sur del muro + nariz,
# sin depender de ninguna distancia medida. La nariz se aleja del muro desde el primer grado.
CORRIDOR_OFF_CM = 0.8          # despegar la nariz del muro antes de pivotear
CORRIDOR_PIVOT_DY = 11.9       # = geo.track_w() * skid / 2: lo que baja el centro al pivotear
CORRIDOR_EXIT_Y = 15.9         # recta de salida: centro de la abertura (y 0.9 .. 30.5)
CORRIDOR_S_DEG = 53            # luego una "S" de dos pivotes hacia adelante (izquierda sobre la oruga
                               # izquierda y derecha sobre la derecha, 53° cada uno) sube el centro
                               # 2 * 11.9 * (1 - cos 53°) = 9.5 cm hasta CORRIDOR_EXIT_Y sin desviarse
                               # (antes se corregía en diagonal y con un recorte llegaba tarde)
CORRIDOR_DIAG_X = 218.6        # TODO(pista) x del centro de giro cuando las sondas centrales
                               # ven la diagonal de la línea yendo al este por CORRIDOR_EXIT_Y
CORRIDOR_TURN_X = 223.0        # girar hasta que la cola rebase la punta del muro (x 212.7) y el
                               # barrido del giro (16 cm de radio) no la toque
RAMP2_X = 228.15            # TODO(pista) centro de la rampa blanca (x 216.7 .. 239.6)
RAMP2_LINE_X = 228.6        # TODO(pista) la línea que baja por ella (0.45 cm al este del centro)
RAMP2_BACK_CM = 8.0         # la nariz ya estaba sobre el pie de la rampa
CORRIDOR_RAMP_CM = 5.5         # en diagonal hasta x ~226.6: el giro de 30° deja la esquina a ~1.3 cm de la orilla de la arena
CORRIDOR_TURN_HEAD = 60        # rumbo al girar: las sondas quedan junto a la línea que baja
# Nivel 2: cruzar la zona de obstáculos (sin línea) de la entrada (abajo a la izquierda) a la
# salida (arriba a la derecha). Al girar en el lugar el robot barre ~13 cm y la línea termina a
# 12 cm del muro de abajo, así que el primer giro es suave (lo mostró el simulador).
CORRIDOR = [
    # Los recortes hacen PATINAR las orugas (con tramos por distancia: 5 % con patinaje en el
    # simulador). La odometría encoders + giroscopio (odometry.py) corrige cuando patina UNA oruga;
    # cuando patinan las dos no hay forma de saberlo, así que la referencia es el muro de arriba:
    ("goto", CORRIDOR_TURN[0], CORRIDOR_TURN[1]),        # al sur, sin seguir la diagonal
    ("head", 0),
    ("track", 0, "y", CORRIDOR_ENTRY_Y, 24),            # entra por el centro de la abertura
    # x exacta: con las dos orugas sobre recortes la odometría llegó a quedarse 15 cm corta y el
    # pivote de después barría el muro izquierdo. Se escuadra contra el muro derecho por la recta
    # y = W1 y se retrocede (si patina al retroceder queda más al este: el lado seguro)
    ("square", 0, CORRIDOR_TO_EAST_S, "x", CORRIDOR_EAST_X, None, CORRIDOR_W1[1]),
    ("back_to", "x", CORRIDOR_W1[0] - CORRIDOR_PIVOT_DY, None, 0, CORRIDOR_W1[1]),
    # al norte PIVOTEANDO sobre la oruga izquierda: la cola barre 12.2 cm hacia el muro de abajo
    # (girando en el lugar 16) y el centro avanza 11.9 al este, hasta W1
    ("pivot", -90),
    ("square", -90, CORRIDOR_TO_WALL_S, "y", CORRIDOR_WALL_Y),    # al norte hasta el muro: y exacta
    ("straight", -CORRIDOR_OFF_CM, "v_slow"),            # despegar la nariz del muro
    ("pivot", 0, "back"),                                # al este, hacia atrás sobre la oruga izquierda
    ("pivot", -CORRIDOR_S_DEG), ("pivot", 0),            # "S": a la recta de salida
    # al este por el centro de la abertura hasta ver la diagonal: fija la x (el error de x de
    # los giros anteriores llegaba a 7 cm y decidía dónde girar)
    ("track_line", 0, "y", CORRIDOR_EXIT_Y, 60),
    ("fix", "x", CORRIDOR_DIAG_X),
    ("track", 0, "y", CORRIDOR_EXIT_Y, CORRIDOR_TURN_X - CORRIDOR_DIAG_X),
    # fuera del pasillo (sin recortes) ya se puede girar en el lugar: sondas junto a la línea
    ("head", CORRIDOR_TURN_HEAD),
    # alinearse con el centro de la rampa ANTES de subir: entrando en curva la cola giraba y una
    # oruga salía por la orilla de la rampa
    ("drive", CORRIDOR_TURN_HEAD, CORRIDOR_RAMP_CM, "v_slow"), ("head", 90),
    ("fix_line", "x", RAMP2_LINE_X, 1),                  # las sondas sobre la línea: x exacta
    ("straight", -RAMP2_BACK_CM, "v_slow"),              # plano para centrarse antes de subir
]
L3_IN_CM = 19.2             # nivel 3: de la orilla de la zona azul a quedar completamente dentro
# Nivel 3 POR MANIOBRA (estrategia del equipo: los puntos son por dejar el prisma en la zona
# punteada, no por seguir la línea). Rumbos de mapa (0 este, 90 sur, 180 oeste, -90 norte) y
# distancias fijas desde la orilla de la zona azul. TODO(medir en pista): salen del plano.
L3_PRISM_SOUTH = 3.6        # de la orilla de la zona azul a la altura del prisma
L3_PRISM_BACK = 4.0         # retroceso para bajar la garra sin tocar el prisma
L3_PRISM_GRAB = 6.5         # avance al oeste hasta tener el prisma en la garra
L3_PRISM_HEAD = 107         # rumbo hacia la zona punteada (con el prisma en la garra)
L3_PRISM_CARRY = 22.0       # avance con el prisma hasta dejarlo en la zona
L3_PRISM_LEAVE = 8.0        # retroceso después de soltarlo
L3_EDGE = (228.8, 70.8)     # centro de giro al salir de la zona azul (sondas en la orilla)
L3_STAIRS_POINT = (166.1, 96.4)   # al sur del tramo recto de la línea, frente a la escalera
PRISM_MANEUVER = [
    ("edge", "blue", "v_slow", 30, 90),      # salir de la zona azul: referencia fija
    ("pose", L3_EDGE[0], L3_EDGE[1], 90),    # y posición conocida para la odometría (sin recortes
                                             # en el nivel 3 la odometría es precisa)
    ("drive", 90, L3_PRISM_SOUTH),
    ("arm", "h_top"), ("head", 180),         # brazo arriba para girar sin barrer el prisma
    ("straight", -L3_PRISM_BACK, "v_slow"),
    ("claw", "claw_open"), ("arm", "h_prism"),
    ("drive", 180, L3_PRISM_GRAB),           # el prisma entra entre los dedos abiertos
    ("head", L3_PRISM_HEAD),                 # gira con el prisma en la garra
    ("drive", L3_PRISM_HEAD, L3_PRISM_CARRY),
    ("arm", "h_carry"),                      # lo suelta
    ("straight", -L3_PRISM_LEAVE, "v_slow"),
]
# De ahí a la escalera: cruzar la línea de la escalera de forma perpendicular (referencia lateral:
# cada oruga debe ir sobre su tramo) y girar al oeste.
STAIRS_BACK_CM = 5.0         # TODO(pista) retroceso antes de buscar la línea frente a la escalera
TO_STAIRS = [
    # las sondas van 5.2 cm delante del centro: al girar al norte en el punto ya quedan sobre la
    # línea o pasadas; se retrocede para que la busquen hacia adelante (la cola queda lejos de la
    # pelota)
    ("head", -90), ("straight", -STAIRS_BACK_CM, "v_slow"),
    ("cross", "v_slow", 15, -90), ("center",),
    ("head", 180),
    # los ~15 cm de línea recta hasta la escalera: el seguidor corrige el desvío lateral y el
    # rumbo promedio queda como referencia para subir (cada oruga debe ir sobre su tramo)
    ("line_ref", 180, ("pitch",), "v_slow", 10),
]
# Reintento del nivel 3 con el prisma ya en la zona: de la zona azul directo al punto frente a la
# escalera (pasa al norte del prisma).
L3_RETRY_TO_STAIRS = [
    ("ref", 90),
    ("edge", "blue", "v_slow", 30, 90),
    ("pose", L3_EDGE[0], L3_EDGE[1], 90),
    ("goto", L3_STAIRS_POINT[0], L3_STAIRS_POINT[1]),
]
# Pelota (misión 8): OMITIDA por ahora (decisión del equipo). Pasos guardados para después.
BALL_HEAD = 113
LINE_HEAD_AT_MARK2 = -152
BALL_STEPS = [("line", ("color", "red"), "v_slow", 20), ("center",),
              ("head", BALL_HEAD), ("arm", "h_pedestal"), ("claw", "claw_open"),
              ("approach_obj", "h_pedestal", "ball_ped_dist"), ("claw", "claw_ball"),
              ("arm", "h_carry"), ("retreat",), ("hold",), ("head", LINE_HEAD_AT_MARK2)]
STAIRS_TOP_CM = 8.0         # nivel 4: avance desde que las sondas ven la zona verde (TODO pista)
# Nivel 4: rodear la torre por el sur (al norte está la barra). Se vuelve a la línea con espacio
# para girar sin barrer la torre (~13 cm de radio) y entre dos tarjetas (lo mostró el simulador).
# Al volver a la línea la nariz (16 cm de radio al girar) debe librar la torre: centro a >= 17 cm
# de su cara oeste. La sonda queda 5.2 cm adelante: la tarjeta más cercana a la torre queda
# detrás y se lee retrocediendo CARDS_BACK_CM (misión 11).
# PENDIENTE (simulador con el contorno medido): entre la cara sur de la torre (y 96.5) y la orilla
# de la arena (y 122) hay 25.5 cm y el robot mide 21.35 (2 cm por lado). El giro al oeste en
# y ~111 saca la nariz ~5 cm de la arena y la recta pasa a 0.2 cm de la orilla; ir al sur desde
# la cima deja la oruga derecha sobre el escalón de 20 cm del nivel 3. Hay que rediseñar este rodeo.
TOWER_WEST_CM = 37.0        # TODO(pista)
CARDS_BACK_CM = 7.0         # TODO(pista) la cola queda a ~1.5 cm de la torre
TOWER_DETOUR = [
    ("drive", 110, 20),         # sur-suroeste: lejos de la orilla del nivel 4 y, al girar al
                                # oeste, a ~2 cm de la esquina de la torre (con 17 la tocaba)
    ("drive", 180, TOWER_WEST_CM),  # oeste, paralelo a la línea, hasta pasar la torre (vuelve
                                    # a la línea lo más cerca posible: la 1a tarjeta está ahí)
    ("seek", "v_slow", 25, -90),  # norte hasta la línea
    ("center",), ("head", 180),  # mirando al oeste sobre la línea (con IMU: las sondas pueden
                                 # quedar sobre una tarjeta que tapa la línea)
]        # nivel 4: avance después de la cima de la escalera (TODO pista)


def junctions(route):
    """Tabla (cruces, tipo, giro) -> pasos. Ya no la usa el nivel 1; queda para rutas por línea."""
    steps = []
    for n, kind, deg in route:
        steps.append(("line", ("junction", n, kind), "v_line", 10))
        steps.append(("jturn", deg))
    return steps


MISSIONS = [
    # ------------------------------------------------------------ NIVEL 1 (zona roja)
    {"id": 1, "level": 1, "pts": 50, "name": "Seguir linea", "critical": True,
     # termina en el cruce frente al cubo (es lo que pide la misión 1)
     # brazo arriba (garra sobre la nariz) mientras gira: con el brazo bajo y al frente la garra
     # barre ~21 cm de radio y tocaba la barra de pelotas
     "steps": [("arm", "h_top"), ("claw", "claw_open"),
               ("ref", 90),                                   # colocado mirando al sur
               ("pose", L1_START[0], L1_START[1], 90),
               # cada tramo termina en una línea que cruza el camino: el error se reinicia ahí
               ("track", 90, "x", L1_TURN_X, L1_SOUTH_CM - L1_CROSS_SLACK, "v_line"),
               ("cross", "v_slow", 2 * L1_CROSS_SLACK, 90),
               ("fix", "y", L1_CROSS_CY),                     # la línea inferior da la y
               ("straight", -L1_TURN_BACK_CM, "v_slow"),
               ("head", 0),                                   # girar YA (no sobre el cruce)
               ("track", 0, "y", L1_CUBE_Y, L1_EAST_CM - L1_CROSS_SLACK - (L1_TURN_X - L1_START[0]),
                "v_line"),
               ("cross", "v_slow", 2 * L1_CROSS_SLACK, 0)]},
    {"id": 2, "level": 1, "pts": 50, "name": "Tomar cubo", "critical": False,
     # primero se coloca (con el brazo arriba) y luego baja la garra abierta sobre el cubo
     "steps": [("approach_ped", "h_pedestal", "ped1_from_cross"), ("arm", "h_pedestal"),
               ("claw", "claw_cube"),
               ("arm", "h_carry"), ("retreat",), ("hold",)]},
    {"id": 3, "level": 1, "pts": 50, "name": "Subir nivel 2", "critical": True,
     # El pedestal está a 8 cm del cruce y el robot barre ~13 cm al girar, así que no puede
     # girar sobre el cruce (lo mostró el simulador). Retrocede, gira al norte, busca la línea
     # punteada y la sigue por la rampa hasta la T del nivel 2 (siguiendo la línea y no recto:
     # así no se pasa la T).
     # Hasta la rampa, por comandos. En la rampa, control activo: sigue la línea (no se cruza en
     # diagonal), baja el brazo al frente (centro de masa más adelante y más bajo) y la potencia
     # sube de a poco mientras va inclinado (hal._tip_guard).
     # Junto al pedestal la pared del nivel 4 queda a 1.6 cm del costado: retrocede más y gira
     # al norte SOBRE LA ORUGA IZQUIERDA (pivote): el costado derecho se aleja de la pared.
     "steps": [("straight", -L1_PIVOT_BACK_CM, "v_slow"), ("pivot", -90),
               ("drive", -90, L1_NORTH_CM - L1_PIVOT_D - (49.0 - L1_CUBE_Y)),
               ("drive", 0, L1_TO_RAMP_CM - L1_PIVOT_D + (L1_PIVOT_BACK_CM - L1_BACK_CM)),
               ("line", ("pitch",), "v_slow", 6), ("arm", "h_climb"),
               ("line", ("junction", 1, "any"), "v_ramp", 8),
               ("center",),                       # centro de giro sobre la T: completamente dentro
               ("arm", "h_carry")]},

    # ------------------------------------------------------------ NIVEL 2 (zona amarilla)
    # La misión 4 es parte del TRAMO 1 ("level" = zona desde la que se arranca): si se pide
    # reinicio en la zona amarilla con el cubo en la garra, el cubo se le quita al robot. Por eso
    # el tramo 1 termina al dejar el cubo en el pedestal del nivel 2, y ahí se pide el reinicio.
    {"id": 4, "level": 1, "pts": 100, "name": "Dejar cubo", "critical": False, "end_level": True,
     # ("ref", 0): viene de seguir 50 cm de línea recta hacia el este por la rampa (o se colocó
     # mirando al este al reiniciar aquí); es la mejor referencia de rumbo para el pasillo.
     # La nariz de las orugas va 8.85 cm delante de las sondas y el cruce está a ~8 cm del
     # pedestal: seguir la línea HASTA el cruce choca la nariz con el pedestal. Se sigue la línea
     # una distancia fija desde la T (garra sobre el centro del pedestal, nariz a ~2 cm de él).
     "steps": [("ref", 0), ("jturn", -90), ("line", ("cm", L2_TO_PED_CM), "v_slow", 8),
               ("arm", "h_pedestal"),
               ("claw", "claw_open"), ("arm", "h_carry"),
               ("straight", -L2_BACK_CM, "v_slow"), ("head", 90),
               ("pose", T2_FROM_M4[0], T2_FROM_M4[1], None)]},
    {"id": 5, "level": 2, "pts": 100, "name": "Zona obstaculos", "critical": True,
     # recién colocado en la zona amarilla (reinicio o arranque en el nivel 2): sondas sobre la
     # T mirando al este, girar al sur hacia el pasillo
     # al girar sobre la T la nariz barre 16 cm y el muro del pasillo está a ~17: se retrocede
     # antes (con ruido chocaba 11 de 12 veces); el giro busca la línea igual
     "steps": [("if_placed", [("ref", 0), ("straight", -L2_T_BACK_CM, "v_slow"), ("jturn", 90),
                              ("pose", T2_PLACED[0], T2_PLACED[1], None)])] + CORRIDOR},
    {"id": 6, "level": 2, "pts": 100, "name": "Subir nivel 3", "critical": True, "end_level": True,
     # sigue la línea por la rampa blanca hasta salir de la zona azul (ya completamente dentro)
     # Al ver la zona azul avanza L3_IN_CM: ya completamente dentro, pero antes de la curva de la
     # S (que empieza dentro de la zona; con "salir de la zona" el robot ya la había pasado).
     # Nunca detenerse sobre la pendiente: al ver la zona azul sigue L3_IN_CM sin parar. El rumbo
     # promedio de toda la rampa (recta al sur) queda como referencia del nivel 3.
     # despacio: la curva de la diagonal a la vertical (47°) se pasaba a v_line (sondas a solo
     # 3 cm del centro de giro: poca anticipación)
     # La rampa mide 22.9 cm y el robot 21.35 (0.78 por lado): siguiendo la línea (que además va
     # 0.45 cm al este del centro) el seguidor oscilaba ±4.5° y una oruga salía hasta 1.3 cm por la
     # orilla. Alineado al pie de la rampa, sube por su centro con giroscopio + odometría.
     "steps": [("arm", "h_climb"),
               ("ramp_up", 90, RAMP2_X, "blue", L3_IN_CM, "v_ramp", 60)]},   # sin detenerse en la pendiente

    # ------------------------------------------------------------ NIVEL 3 (zona azul)
    {"id": 7, "level": 3, "pts": 100, "name": "Mover prisma", "critical": False,
     "steps": [("ref", 90, 3)] + PRISM_MANEUVER},              # reinicio aquí: colocado al sur
    # misión 8 (pelota) omitida: ver BALL_STEPS
    {"id": 9, "level": 3, "pts": 150, "name": "Escalera nivel 4", "critical": True,
     # Entre la escalera y la torre hay ~20 cm y el robot mide ~24 cm con la garra: no cabe
     # completo sobre la línea; al ver la zona verde avanza STAIRS_TOP_CM y el brazo sube sin
     # detenerse (parar en la escalera lo hacía resbalar). Termina de entrar al rodear la torre.
     "steps": [("if_skipped", 7, L3_RETRY_TO_STAIRS),
               ("if_ran", 7, [("goto", L3_STAIRS_POINT[0], L3_STAIRS_POINT[1])])] + TO_STAIRS +
              [("arm", "h_climb"),
                           ("crest", STAIRS_TOP_CM, 180, "green", "h_top")]},

    # ------------------------------------------------------------ NIVEL 4 (zona verde)
    {"id": 10, "level": 4, "pts": 100, "name": "Evitar torre", "critical": True,
     # Los niveles 3 y 4 van seguidos (sin reinicio): el robot no cabe completo entre la escalera
     # y la torre, así que el nivel 3 se completa al rodearla. Tras una falla aquí se reinicia en
     # la zona verde.
     # brazo al máximo (garra recogida 3 cm) antes de rodear: con el brazo a altura de carga la
     # garra barría la torre al girar (lo mostró el simulador). Colocado en la zona verde también
     # hace falta: ahí la garra a altura de carga ya toca la torre.
     "steps": [("ref", 180, 4), ("arm", "h_top"),
              ] + TOWER_DETOUR},
    {"id": 11, "level": 4, "pts": 120, "name": "Colores", "critical": False,
     "steps": [("cards", CARDS_BACK_CM)]},
    # misión 12 (pelota en la casilla roja) omitida: necesita la pelota de la misión 8
    {"id": 13, "level": 4, "pts": 100, "name": "Meta azul", "critical": False,
     "steps": [("stop_in", "blue")]},
]


def total_points():
    t = 0
    for m in MISSIONS:
        t += m["pts"]
    return t
