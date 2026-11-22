# Rover X 2026: misiones del área automática, escritas como listas de pasos.
#
# BORRADOR DE RUTA: se armó con las vistas de la arena del reglamento
# (resources/reglamento/arena/). Los giros, distancias y conteos de cruces se ajustan
# en la pista. La lógica de cada paso está en autonomous.py (run_step).
#
# Pasos disponibles:
#   ("line", cond, vel, timeout, verde)  seguir la línea hasta cond. Cond puede ser:
#        ("cross", n) n intersecciones  ("color", nombre)  ("cm", d)  ("obstacle", cm)
#        ("end",) se acaba la línea     ("pitch",) empieza una rampa
#   ("straight", cm)           recto con giroscopio (negativo = atrás)
#   ("straight_slow", cm)
#   ("turn", grados)           giro relativo, positivo = derecha
#   ("spin", dir)              girar hasta encontrar la línea (1 der, -1 izq)
#   ("approach", cm)           avanzar lento hasta que el ultrasónico marque <= cm
#   ("climb",) / ("stairs",)   subir rampa / escalera
#   ("claw", ángulo) / ("lift", ángulo)
#   ("hold",)                  sostener 5 s (validación de cubo y pelota)
#   ("detour",)                rodear la torre
#   ("cards",)                 leer las 6 tarjetas de color
#   ("drop_red",)              dejar la pelota frente a la tarjeta roja
#   ("stop_in", color)         seguir la línea y parar dentro de la zona de ese color
#
# critical=True: si falla, el robot quedó fuera de ruta y el autónomo se detiene (conviene
# pedir reinicio en la zona del nivel). critical=False: se sigue con la siguiente misión.
# Las misiones de manipulación incluyen su propio camino de ida y vuelta, así que se
# ejecutan aunque falle la anterior (p. ej. M4 sin cubo solo cuesta unos segundos).

import config as cfg

V = cfg.SPEED_LINE
VS = cfg.SPEED_LINE_SLOW

# ---- constantes de ruta (AJUSTAR EN PISTA)
PEDESTAL_SEE_CM = 25        # el ultrasónico ve el pedestal al final del tramo
L1_TURN_TO_RAMP = 90        # giro desde el pedestal del nivel 1 hacia la rampa
L2_TURN_TO_CORRIDOR = 180   # tras dejar el cubo, regresar hacia el pasillo
CORRIDOR_CM = 70            # largo del pasillo de obstáculos sin línea
PRISM_MARK = "red"          # marca de referencia donde queda la zona punteada
BALL_TURN = -90             # el pedestal de la pelota está a la izquierda de la línea

MISSIONS = [
    # ------------------------------------------------------------ NIVEL 1 (zona roja)
    {"id": 1, "level": 1, "pts": 50, "name": "Seguir linea", "critical": True,
     "steps": [("lift", cfg.LIFT_PEDESTAL), ("claw", cfg.CLAW_OPEN),
               ("line", ("obstacle", PEDESTAL_SEE_CM), V, 25, True)]},
    {"id": 2, "level": 1, "pts": 50, "name": "Tomar cubo", "critical": False,
     "steps": [("approach", cfg.PEDESTAL_APPROACH_CM), ("claw", cfg.CLAW_CLOSED),
               ("lift", cfg.LIFT_UP), ("straight_slow", -8), ("hold",)]},
    {"id": 3, "level": 1, "pts": 50, "name": "Subir nivel 2", "critical": True,
     "steps": [("straight", -6), ("turn", L1_TURN_TO_RAMP), ("spin", 1),
               ("line", ("pitch",), V, 8, False), ("climb",)]},

    # ------------------------------------------------------------ NIVEL 2 (zona amarilla)
    {"id": 4, "level": 2, "pts": 100, "name": "Dejar cubo", "critical": False,
     "steps": [("line", ("obstacle", PEDESTAL_SEE_CM), VS, 8, False),
               ("lift", cfg.LIFT_PEDESTAL), ("approach", cfg.PEDESTAL_APPROACH_CM),
               ("claw", cfg.CLAW_OPEN), ("straight_slow", -10), ("lift", cfg.LIFT_UP),
               ("turn", L2_TURN_TO_CORRIDOR), ("spin", 1)]},
    {"id": 5, "level": 2, "pts": 100, "name": "Zona obstaculos", "critical": True,
     "steps": [("line", ("end",), V, 10, False), ("straight_slow", CORRIDOR_CM),
               ("spin", -1)]},
    {"id": 6, "level": 2, "pts": 100, "name": "Subir nivel 3", "critical": True,
     "steps": [("line", ("pitch",), V, 8, False), ("climb",)]},

    # ------------------------------------------------------------ NIVEL 3 (zona azul)
    {"id": 7, "level": 3, "pts": 100, "name": "Mover prisma", "critical": False,
     "steps": [("line", ("color", PRISM_MARK), cfg.SPEED_PUSH, 12, False),
               ("straight_slow", -6), ("detour",)]},
    {"id": 8, "level": 3, "pts": 100, "name": "Tomar pelota", "critical": False,
     "steps": [("line", ("color", "red"), V, 10, False), ("turn", BALL_TURN),
               ("lift", cfg.LIFT_PEDESTAL), ("claw", cfg.CLAW_OPEN),
               ("approach", cfg.PEDESTAL_APPROACH_CM), ("claw", cfg.CLAW_BALL),
               ("lift", cfg.LIFT_UP), ("straight_slow", -8), ("hold",),
               ("turn", -BALL_TURN), ("spin", 1)]},
    {"id": 9, "level": 3, "pts": 150, "name": "Escalera nivel 4", "critical": True,
     "steps": [("line", ("pitch",), V, 10, False), ("stairs",)]},

    # ------------------------------------------------------------ NIVEL 4 (zona verde)
    {"id": 10, "level": 4, "pts": 100, "name": "Evitar torre", "critical": True,
     "steps": [("line", ("obstacle", cfg.TOWER_DETECT_CM), V, 10, False), ("detour",)]},
    {"id": 11, "level": 4, "pts": 120, "name": "Colores", "critical": False,
     "steps": [("cards",)]},
    {"id": 12, "level": 4, "pts": 150, "name": "Pelota en rojo", "critical": False,
     "steps": [("drop_red",)]},
    {"id": 13, "level": 4, "pts": 100, "name": "Meta azul", "critical": False,
     "steps": [("stop_in", cfg.GOAL_COLOR)]},
]


def total_points():
    t = 0
    for m in MISSIONS:
        t += m["pts"]
    return t
