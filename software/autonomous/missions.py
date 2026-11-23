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

# ---- constantes de ruta (TODO: AJUSTAR EN PISTA; salen del plano y se probaron en el simulador)
L1_BACK_CM = 4.5            # nivel 1: retroceso tras el cubo para poder girar sin barrer el pedestal
L2_TO_PED_CM = 9.6          # TODO(pista) nivel 2: línea desde la T hasta la garra sobre el pedestal
L2_T_BACK_CM = 3.0          # TODO(pista) nivel 2: retroceso en la T antes de girar al sur
L2_BACK_CM = 6.0            # nivel 2: lo mismo frente al pedestal donde se deja el cubo
# Rumbos de mapa (pasos "ref" y "head"): 0 = este, 90 = sur, 180 = oeste, -90 = norte, en la
# vista superior del reglamento. Se fijan con "ref" sobre una recta conocida.
# Puntos del mapa en cm (x este desde la orilla izquierda, y SUR desde la orilla de arriba),
# del plano reconstruido (simulation/arena.py). TODO(medir en pista).
# El robot mide 21.35 de ancho y la línea entra a ~11 cm del muro de abajo: seguirla hasta su fin
# dejaba ~0.6 cm y el seguidor, al salir de la curva, tocaba la esquina. Se deja la línea a media
# curva (rumbo CORRIDOR_START_HEAD) y se entra por el centro de la abertura con odometría.
CORRIDOR_START_HEAD = 45         # rumbo de mapa a media curva (de sur a este)
CORRIDOR_START = (135.9, 44.6)   # TODO(pista) centro de giro al pasar por ese rumbo
CORRIDOR_W1 = (183.0, 42.5)      # TODO(pista) a 17 cm del muro de abajo (la nariz barre 16 cm al girar).
                                 # Los recortes junto al muro de arriba (x 175..190) ocupan 15.3 cm, casi el
                                 # hueco entre orugas (15.35): ningún punto deja las orugas fuera al girar
CORRIDOR_TO_WALL_S = 6.0         # s al norte a v_slow: ~35 cm hasta el muro con patinaje, y empuja
CORRIDOR_WALL_Y = 0.9 + 11.85    # y del centro de giro con la nariz en el muro (geo.track_front())
CORRIDOR_BACK_CM = 7.0         # retroceso para girar al este: la nariz barre 16 cm de radio (muro a >= 0.5)
CORRIDOR_EXIT_DY = 3.25        # recta de salida a y 16: centro de la abertura y toma la diagonal de la línea
# Nivel 2: cruzar la zona de obstáculos (sin línea) de la entrada (abajo a la izquierda) a la
# salida (arriba a la derecha). Al girar en el lugar el robot barre ~13 cm y la línea termina a
# 12 cm del muro de abajo, así que el primer giro es suave (lo mostró el simulador).
CORRIDOR = [
    # Los recortes hacen PATINAR las orugas (con tramos por distancia: 5 % con patinaje en el
    # simulador). La odometría encoders + giroscopio (odometry.py) corrige cuando patina UNA oruga;
    # cuando patinan las dos no hay forma de saberlo, así que la referencia es el muro de arriba:
    ("pose", CORRIDOR_START[0], CORRIDOR_START[1], None),  # a media curva (rumbo: el del IMU)
    ("goto", CORRIDOR_W1[0], CORRIDOR_W1[1]),            # lejos del muro de abajo
    ("square", -90, CORRIDOR_TO_WALL_S, "y", CORRIDOR_WALL_Y),    # al norte hasta el muro: y exacta
    ("straight", -CORRIDOR_BACK_CM, "v_slow"),           # espacio para girar
    # terminar listo para seguir la línea: al este por la recta y = muro + retroceso hasta la
    # diagonal (absorbe el error a lo largo y corrige el lateral)
    ("track_line", 0, "y", CORRIDOR_WALL_Y + CORRIDOR_EXIT_DY, 90),
    # tomar la diagonal con el seguidor MUY despacio, sin girar en el lugar: un giro con una
    # oruga sobre un recorte corre el robot hasta 4 cm (la odometría no lo puede ver) y cerca
    # del muro de arriba lo hacía chocar
    ("line", ("cm", 10), "v_turn_min", 8),
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
               # cada tramo termina en una línea que cruza el camino: el error se reinicia ahí
               ("drive", 90, L1_SOUTH_CM - L1_CROSS_SLACK, "v_line"),
               ("cross", "v_slow", 2 * L1_CROSS_SLACK, 90), ("center",),
               ("drive", 0, L1_EAST_CM - L1_CROSS_SLACK, "v_line"),
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
     "steps": [("straight", -L1_BACK_CM, "v_slow"),
               ("drive", -90, L1_NORTH_CM), ("drive", 0, L1_TO_RAMP_CM),
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
               ("straight", -L2_BACK_CM, "v_slow"), ("head", 90)]},
    {"id": 5, "level": 2, "pts": 100, "name": "Zona obstaculos", "critical": True,
     # recién colocado en la zona amarilla (reinicio o arranque en el nivel 2): sondas sobre la
     # T mirando al este, girar al sur hacia el pasillo
     # al girar sobre la T la nariz barre 16 cm y el muro del pasillo está a ~17: se retrocede
     # antes (con ruido chocaba 11 de 12 veces); el giro busca la línea igual
     "steps": [("if_placed", [("ref", 0), ("straight", -L2_T_BACK_CM, "v_slow"), ("jturn", 90)]),
               ("line", ("head", CORRIDOR_START_HEAD, 6), "v_slow", 15)] + CORRIDOR},
    {"id": 6, "level": 2, "pts": 100, "name": "Subir nivel 3", "critical": True, "end_level": True,
     # sigue la línea por la rampa blanca hasta salir de la zona azul (ya completamente dentro)
     # Al ver la zona azul avanza L3_IN_CM: ya completamente dentro, pero antes de la curva de la
     # S (que empieza dentro de la zona; con "salir de la zona" el robot ya la había pasado).
     # Nunca detenerse sobre la pendiente: al ver la zona azul sigue L3_IN_CM sin parar. El rumbo
     # promedio de toda la rampa (recta al sur) queda como referencia del nivel 3.
     # despacio: la curva de la diagonal a la vertical (47°) se pasaba a v_line (sondas a solo
     # 3 cm del centro de giro: poca anticipación)
     "steps": [("line", ("pitch",), "v_slow", 16), ("arm", "h_climb"),
               ("line_ref", 90, ("color_cm", "blue", L3_IN_CM), "v_ramp", 20)]},   # 20 s: subida lenta con asistencia

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
