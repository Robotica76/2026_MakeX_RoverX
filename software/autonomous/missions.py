# Rover X 2026: misiones del área automática, escritas como listas de pasos.
#
# BORRADOR DE RUTA: se armó con las vistas de la arena del reglamento
# (resources/reglamento/arena/). Los conteos de cruces, giros y distancias se ajustan en la
# pista. La lógica de cada paso está en autonomous.py (run_step).
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
#   ("turn", grados)           giro relativo, positivo = derecha
#   ("spin", dir)              girar hasta encontrar la línea (1 der, -1 izq)
#   ("approach_ped", altura, dist)  dejar la garra (a esa altura) sobre el centro de un pedestal
#                              que está a `dist` cm del último cruce (medida de la pista)
#   ("approach_obj", altura, dist)  igual, para un objeto a `dist` cm del centro de giro
#   ("retreat",)               regresar lo mismo que avanzó el último acercamiento
#   ("to_tower",)              seguir la línea hasta quedar junto a la torre (desde la zona verde)
#
# El robot NO lleva ultrasónico: los acercamientos se hacen con el encoder, desde un cruce o
# el borde de una zona, usando medidas de la pista (cfg.P "ped1_from_cross", etc.).
#   ("climb",) / ("stairs",)   subir rampa / escalera
#   ("claw", apertura) / ("arm", altura)
#   ("hold",)                  sostener 5 s (validación de cubo y pelota)
#   ("detour", lado)           rodear la torre (-1 izquierda, 1 derecha)
#   ("cards",) / ("drop_red",) / ("stop_in", color)
#
# critical=True: si falla, el robot quedó fuera de ruta y el autónomo se detiene (conviene
# pedir reinicio en la zona del nivel). critical=False: se sigue con la siguiente misión.

# ---- ruta del nivel 1: (cuántos cruces, tipo, giro al llegar). CONTAR EN PISTA.
# Desde la X roja hasta el tramo que lleva al pedestal del cubo. Giro 0 = seguir derecho.
ROUTE_L1 = [
    (1, "any", -90),
    (1, "any", -90),
    (1, "any", 90),
]

# ---- constantes de ruta (AJUSTAR EN PISTA)
L1_TURN_TO_RAMP = 90        # giro desde el pedestal del nivel 1 hacia la rampa
L2_TURN_TO_CORRIDOR = 180   # tras dejar el cubo, regresar hacia el pasillo
CORRIDOR_CM = 70            # largo del pasillo de obstáculos sin línea
PRISM_MARK = "red"          # marca de referencia donde queda la zona punteada
BALL_TURN = -90             # el pedestal de la pelota está a la izquierda de la línea
TOWER_SIDE = -1             # rodear la torre por la izquierda


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
              [("line", ("junction", 1, "any"), "v_line", 10)]},
    {"id": 2, "level": 1, "pts": 50, "name": "Tomar cubo", "critical": False,
     "steps": [("approach_ped", "h_pedestal", "ped1_from_cross"), ("claw", "claw_cube"),
               ("arm", "h_carry"), ("retreat",), ("hold",)]},
    {"id": 3, "level": 1, "pts": 50, "name": "Subir nivel 2", "critical": True,
     "steps": [("straight", -6), ("turn", L1_TURN_TO_RAMP), ("spin", 1),
               ("line", ("pitch",), "v_line", 8), ("climb",)]},

    # ------------------------------------------------------------ NIVEL 2 (zona amarilla)
    {"id": 4, "level": 2, "pts": 100, "name": "Dejar cubo", "critical": False,
     "steps": [("line", ("junction", 1, "any"), "v_slow", 8),
               ("arm", "h_pedestal"), ("approach_ped", "h_pedestal", "ped2_from_cross"),
               ("claw", "claw_open"), ("retreat",), ("arm", "h_carry"),
               ("turn", L2_TURN_TO_CORRIDOR), ("spin", 1)]},
    {"id": 5, "level": 2, "pts": 100, "name": "Zona obstaculos", "critical": True,
     "steps": [("line", ("end",), "v_line", 10), ("straight", CORRIDOR_CM, "v_slow"),
               ("spin", -1)]},
    {"id": 6, "level": 2, "pts": 100, "name": "Subir nivel 3", "critical": True,
     "steps": [("line", ("pitch",), "v_line", 8), ("climb",)]},

    # ------------------------------------------------------------ NIVEL 3 (zona azul)
    {"id": 7, "level": 3, "pts": 100, "name": "Mover prisma", "critical": False,
     "steps": [("line", ("color", PRISM_MARK), "v_push", 12),
               ("straight", -6, "v_slow"), ("detour", TOWER_SIDE)]},
    {"id": 8, "level": 3, "pts": 100, "name": "Tomar pelota", "critical": False,
     "steps": [("line", ("color", "red"), "v_line", 10),
               ("straight", "sensor_fwd", "v_slow"),          # centro de giro sobre la marca
               ("turn", BALL_TURN), ("arm", "h_pedestal"), ("claw", "claw_open"),
               ("approach_obj", "h_pedestal", "ball_ped_dist"), ("claw", "claw_ball"),
               ("arm", "h_carry"), ("retreat",), ("hold",),
               ("turn", -BALL_TURN), ("spin", 1)]},
    {"id": 9, "level": 3, "pts": 150, "name": "Escalera nivel 4", "critical": True,
     "steps": [("line", ("pitch",), "v_line", 10), ("stairs",)]},

    # ------------------------------------------------------------ NIVEL 4 (zona verde)
    {"id": 10, "level": 4, "pts": 100, "name": "Evitar torre", "critical": True,
     "steps": [("line", ("leave", "green"), "v_slow", 6), ("to_tower",),
               ("detour", TOWER_SIDE)]},
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
