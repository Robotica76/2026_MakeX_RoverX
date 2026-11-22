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
#   ("spin", dir)              girar hasta encontrar la línea (1 der, -1 izq)
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
#   ("if_start", nivel, [pasos])  pasos que solo se hacen si el autónomo arrancó en ese nivel
#   ("crest", cm)              subir hasta pasar la cima y avanzar cm (poco espacio arriba)
#   ("claw", apertura) / ("arm", altura)
#   ("hold",)                  sostener 5 s (validación de cubo y pelota)
#   ("detour", lado)           rodear la torre (-1 izquierda, 1 derecha)
#   ("cards",) / ("drop_red",) / ("stop_in", color)
#
# critical=True: si falla, el robot quedó fuera de ruta y el autónomo se detiene (conviene
# pedir reinicio en la zona del nivel). critical=False: se sigue con la siguiente misión.

# ---- ruta del nivel 1: (cuántos cruces, tipo, giro al llegar). CONTAR EN PISTA.
# Plano del reglamento (simulation/arena.py): de la X roja se baja por la vertical hasta el cruce
# de la línea inferior y se gira a la izquierda; ahí se siguen 2 cruces (la T de la vertical
# corta y el cruce frente al pedestal, pasando el tope gris). Giro 0 = seguir derecho.
ROUTE_L1 = [
    (1, "any", -90),
]
L1_CROSSES_TO_PEDESTAL = 2

# ---- constantes de ruta (TODO: AJUSTAR EN PISTA; salen del plano y se probaron en el simulador)
L1_BACK_CM = 2.0            # nivel 1: retroceso tras el cubo para poder girar sin barrer el pedestal
L2_BACK_CM = 3.5            # nivel 2: lo mismo frente al pedestal donde se deja el cubo
# Rumbos de mapa (pasos "ref" y "head"): 0 = este, 90 = sur, 180 = oeste, -90 = norte, en la
# vista superior del reglamento. Se fijan con "ref" sobre una recta conocida.
# Nivel 2: cruzar la zona de obstáculos (sin línea) de la entrada (abajo a la izquierda) a la
# salida (arriba a la derecha). Al girar en el lugar el robot barre ~13 cm y la línea termina a
# 12 cm del muro de abajo, así que el primer giro es suave (lo mostró el simulador).
CORRIDOR = [
    # ("drive", rumbo de mapa, cm): cada tramo SOSTIENE su rumbo con el IMU, así el error del
    # giro y los golpes de los recortes se corrigen mientras avanza.
    ("drive", 0, 2),            # line_end ya avanzó 2 cm sin línea
    ("drive", -15, 12.6),
    ("drive", -45, 40.5),
    ("seek", "v_slow", 35, 0),  # hacia el este hasta la diagonal que baja a la rampa blanca
    ("center",), ("spin", 1),   # centro de giro sobre la diagonal y girar a seguirla hacia abajo
]
L3_IN_CM = 12.0             # nivel 3: de la orilla de la zona azul a quedar completamente dentro
# Nivel 3: rodear el prisma ya dejado en la zona punteada por el norte (al sur está la orilla de
# la arena) y volver a la línea antes de la segunda marca roja. Rumbos de mapa, referidos a la
# recta de la rampa blanca (hacia el sur).
PRISM_DETOUR = [
    ("arm", "h_carry"),         # suelta el prisma
    ("straight", -6, "v_slow"),
    ("drive", -90, 9.5),        # norte: separarse del prisma
    ("drive", 180, 35),         # oeste: rebasarlo
    ("seek", "v_slow", 25, 135),  # suroeste hasta la línea que sube a la segunda marca roja
    ("center",), ("spin", 1),
]
BALL_HEAD = 113             # nivel 3: rumbo de mapa de la segunda marca roja al pedestal de la pelota
LINE_HEAD_AT_MARK2 = -152   # rumbo de la línea en la segunda marca roja (hacia la escalera)
STAIRS_TOP_CM = 7.0         # nivel 4: avance desde que las sondas ven la zona verde (TODO pista)
# Nivel 4: rodear la torre por el sur (al norte está la barra). Se vuelve a la línea con espacio
# para girar sin barrer la torre (~13 cm de radio) y entre dos tarjetas (lo mostró el simulador).
TOWER_DETOUR = [
    ("drive", 110, 17),         # sur-suroeste: alejarse de la orilla del nivel 4 (a ~4 cm)
    ("drive", 180, 41),         # oeste, paralelo a la línea, hasta pasar la torre
    ("seek", "v_slow", 25, -90),  # norte hasta la línea
    ("center",), ("spin", -1),  # quedar mirando al oeste sobre la línea
]        # nivel 4: avance después de la cima de la escalera (TODO pista)


def junctions(route):
    steps = []
    for n, kind, deg in route:
        steps.append(("line", ("junction", n, kind), "v_line", 10))
        steps.append(("jturn", deg))
    return steps


MISSIONS = [
    # ------------------------------------------------------------ NIVEL 1 (zona roja)
    {"id": 1, "level": 1, "pts": 50, "name": "Seguir linea", "critical": True,
     # termina en el cruce frente al cubo (es lo que pide la misión 1)
     "steps": [("arm", "h_pedestal"), ("claw", "claw_open")] + junctions(ROUTE_L1) +
              [("line", ("junction", L1_CROSSES_TO_PEDESTAL, "any"), "v_line", 12)]},
    {"id": 2, "level": 1, "pts": 50, "name": "Tomar cubo", "critical": False,
     "steps": [("approach_ped", "h_pedestal", "ped1_from_cross"), ("claw", "claw_cube"),
               ("arm", "h_carry"), ("retreat",), ("hold",)]},
    {"id": 3, "level": 1, "pts": 50, "name": "Subir nivel 2", "critical": True,
     # El pedestal está a 8 cm del cruce y el robot barre ~13 cm al girar, así que no puede
     # girar sobre el cruce (lo mostró el simulador). Retrocede, gira al norte, busca la línea
     # punteada y la sigue por la rampa hasta la T del nivel 2 (siguiendo la línea y no recto:
     # así no se pasa la T).
     "steps": [("straight", -L1_BACK_CM, "v_slow"), ("turn", -90), ("seek", "v_slow", 25),
               ("center",), ("turn", 90), ("line", ("pitch",), "v_line", 10),
               ("line", ("junction", 1, "any"), "v_ramp", 8)]},

    # ------------------------------------------------------------ NIVEL 2 (zona amarilla)
    {"id": 4, "level": 2, "pts": 100, "name": "Dejar cubo", "critical": False,
     # ("ref", 0): viene de seguir 50 cm de línea recta hacia el este por la rampa (o se colocó
     # mirando al este al reiniciar aquí); es la mejor referencia de rumbo para el pasillo.
     "steps": [("ref", 0), ("jturn", -90), ("line", ("junction", 1, "any"), "v_slow", 8),
               ("arm", "h_pedestal"), ("approach_ped", "h_pedestal", "ped2_from_cross"),
               ("claw", "claw_open"), ("arm", "h_carry"), ("retreat",),
               ("straight", -L2_BACK_CM, "v_slow"), ("head", 90)]},
    {"id": 5, "level": 2, "pts": 100, "name": "Zona obstaculos", "critical": True,
     "steps": [("line", ("end",), "v_slow", 15)] + CORRIDOR},
    {"id": 6, "level": 2, "pts": 100, "name": "Subir nivel 3", "critical": True,
     # sigue la línea por la rampa blanca hasta salir de la zona azul (ya completamente dentro)
     # Al ver la zona azul avanza L3_IN_CM: ya completamente dentro, pero antes de la curva de la
     # S (que empieza dentro de la zona; con "salir de la zona" el robot ya la había pasado).
     "steps": [("line", ("pitch",), "v_line", 12),
               ("line", ("color", "blue"), "v_ramp", 8),
               ("ref", 90),                       # después de 30 cm de recta en la rampa, al sur
               ("line", ("cm", L3_IN_CM), "v_slow", 4)]},

    # ------------------------------------------------------------ NIVEL 3 (zona azul)
    {"id": 7, "level": 3, "pts": 100, "name": "Mover prisma", "critical": False,
     # Empujarlo con el frente no sirve: en la primera curva se sale de la línea, y para empujarlo
     # hacia la zona el robot no cabe detrás de él (lo mostró el simulador). Se lleva con la garra
     # abierta y baja, como una cuna, siguiendo la línea; en la primera marca roja queda dentro de
     # la zona punteada. Se sube el brazo, se retrocede y se rodea.
     "steps": [("ref", 90, 3),                                # reinicio aquí: colocado al sur
               ("claw", "claw_open"), ("arm", "h_prism"),
               ("line", ("color", "red"), "v_push", 25)] + PRISM_DETOUR},
    {"id": 8, "level": 3, "pts": 100, "name": "Tomar pelota", "critical": False,
     "steps": [("line", ("color", "red"), "v_slow", 20),         # despacio: la marca va junto a la línea
               ("center",),                                   # centro de giro sobre la marca
               ("head", BALL_HEAD), ("arm", "h_pedestal"), ("claw", "claw_open"),
               ("approach_obj", "h_pedestal", "ball_ped_dist"), ("claw", "claw_ball"),
               ("arm", "h_carry"), ("retreat",), ("hold",),
               ("head", LINE_HEAD_AT_MARK2)]},
    {"id": 9, "level": 3, "pts": 150, "name": "Escalera nivel 4", "critical": True,
     # Entre la escalera y la torre hay ~20 cm y el robot mide ~24 cm con la garra: no cabe
     # completo sobre la línea. Al ver la zona verde avanza STAIRS_TOP_CM (garra a ~4 cm de la torre) y
     # termina de entrar al nivel 4 al rodear la torre (lo mostró el simulador).
     # La escalera tiene dos tramos con 12 cm de hueco: se llega despacio (el seguidor queda
     # centrado) y se sube sosteniendo el rumbo oeste con el IMU.
     "steps": [("line", ("cm", 11), "v_slow", 6),            # hasta el tramo recto final
               ("line_ref", 180, ("pitch",), "v_slow", 10),     # rumbo de la recta = oeste
               ("crest", STAIRS_TOP_CM, 180, "green")]},

    # ------------------------------------------------------------ NIVEL 4 (zona verde)
    {"id": 10, "level": 4, "pts": 100, "name": "Evitar torre", "critical": True,
     # Al reiniciar en la zona verde, primero se acerca a la torre; viniendo de la escalera ya
     # quedó a la distancia correcta.
     "steps": [("ref", 180, 4),
               ("if_start", 4, [("line", ("leave", "green"), "v_slow", 6), ("to_tower",)]),
              ] + TOWER_DETOUR},
    {"id": 11, "level": 4, "pts": 120, "name": "Colores", "critical": False,
     "steps": [("cards",)]},
    {"id": 12, "level": 4, "pts": 150, "name": "Pelota en rojo", "critical": False,
     "steps": [("drop_red",)]},
    {"id": 13, "level": 4, "pts": 100, "name": "Meta azul", "critical": False,
     "steps": [("stop_in", "blue")]},
]


def total_points():
    t = 0
    for m in MISSIONS:
        t += m["pts"]
    return t
