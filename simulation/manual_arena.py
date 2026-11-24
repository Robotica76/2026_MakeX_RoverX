"""Arena MANUAL de Rover X (misiones de alianza) para el simulador.

Reconstruida de la vista superior del reglamento
(resources/reglamento/arena/arena_manual_superior.png, 814 x 427 px). La arena mide
2440 x 1220 mm (la otra mitad de la pista de 2440 x 2440): 3 tableros a lo ancho y 2 a lo alto.
En la imagen la arena va de x = 12 a 788 px y de y = 26 a 404 px.

- Nivel 1 (piso): tableros de la izquierda y de la derecha.
- Nivel 2 (20 cm, lo que suben las escaleras de 8 x 2.5 cm): la columna del centro.
- Escaleras: dos dobles (dos tramos separados 12 cm), una a cada lado, suben hacia el centro.
- Rampas abatibles: empiezan LEVANTADAS (de pie en la orilla del nivel 2, como un puente
  levadizo). Desde el nivel 2 el robot las empuja y caen hacia afuera: quedan como rampa
  (misión de alianza 2, 30 pts c/u + 40 si son las dos).
- Barra MakeX (magenta) arriba del nivel 2 con 5 lugares para los cubos M-A-K-E-X.
- Cubos con letra sobre pedestales (6 x 6 x 10 cm); 4 cilindros (5 x 15 cm) en las esquinas del
  tablero rojo/morado; el área negra es el círculo del centro de ese tablero.

Medidas DEL REGLAMENTO: pista 2440 x 2440 mm (esta mitad: 2440 x 1220, 3 tableros de 813 x 610 mm),
escalera de 8 peldaños de 2.5 cm (nivel 2 = 20 cm) con huella de 3.7 cm y 12 cm entre tramos,
rampas de 22 cm de ancho mínimo y 40° máximo (aquí 23 cm y ~33°), pedestales de 6 x 6 x 10 cm,
cubos de 50 mm, cilindros de 5 cm de diámetro x 15 cm, huecos de hasta 10 mm.
Medidas DEL DIBUJO A ESCALA (el reglamento no las da): posición de todo, círculo negro (~19 cm de
radio), barra MakeX (57 x 6 cm; alto supuesto 10 cm como la barra de pelotas) y el mecanismo
de las rampas. TODO(pista): confirmarlas en la pista real.

Marco en cm: x hacia la derecha de la imagen, y hacia ARRIBA, origen en la esquina inferior
izquierda.
"""

import math

import arena as ar

SX = 244.0 / (788 - 12)          # cm por pixel a lo ancho
SY = 122.0 / (404 - 26)          # cm por pixel a lo alto
W_CM = 244.0
H_CM = 122.0


def px(x, y):
    return ((x - 12) * SX, (404 - y) * SY)


def rect_px(x0, y0, x1, y1):
    a, b = px(x0, y0), px(x1, y1)
    return (min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1]))


X_L2 = (271 - 12) * SX           # orilla izquierda del nivel 2 (81.4 cm)
X_R2 = (529 - 12) * SX           # orilla derecha (162.6 cm)
Z = {1: 0.0, 2: 20.0}            # escalera de 8 x 2.5 cm

PEDESTAL = 6.0
PEDESTAL_H = 10.0
CUBE = 5.0
CYL_D = 5.0
CYL_H = 15.0
BAR_H = 10.0                     # TODO(pista) alto de la barra MakeX
LETTERS = "MAKEX"

STAIR_STEPS = 8
STAIR_RISE = 2.5
# dos tramos por escalera (y en px: 83-101 y 140-158); la izquierda sube hacia +x, la derecha hacia -x
FLIGHTS_Y_PX = [(83, 101), (140, 158)]
STAIR_RUN = STAIR_STEPS * 3.7    # huella de 3.7 cm


def _flight_rects(side):
    out = []
    for y0, y1 in FLIGHTS_Y_PX:
        _, ya = px(0, y1)
        _, yb = px(0, y0)
        if side < 0:
            out.append((X_L2 - STAIR_RUN, ya, X_L2, yb))
        else:
            out.append((X_R2, ya, X_R2 + STAIR_RUN, yb))
    return out


# rampas: rectángulo sobre el nivel 1 (bisagra en la orilla del nivel 2); ancho 23 cm
RAMPS = [
    {"name": "izquierda", "rect": (rect_px(169, 274, 266, 346)[0], rect_px(169, 274, 266, 346)[1],
                                   X_L2, rect_px(169, 274, 266, 346)[3]), "hinge_x": X_L2, "out": -1},
    {"name": "derecha", "rect": (X_R2, rect_px(533, 274, 631, 346)[1],
                                 rect_px(533, 274, 631, 346)[2], rect_px(533, 274, 631, 346)[3]),
     "hinge_x": X_R2, "out": 1},
]
RAMP_UP_H = 30.0                 # alto de la rampa levantada sobre el nivel 2 (su largo)

PEDESTALS_PX = [(49, 38), (750.5, 38), (258, 203), (399, 203), (541.5, 203)]
CYLINDERS_PX = [(286.5, 249.5), (513, 249.5), (286.5, 388.5), (513, 388.5)]
BAR_PX = (309, 31, 491, 49)
SLOTS_PX = [325.5, 362, 399.5, 437.5, 474.5]
DISC_PX = (400, 319.5)
DISC_R = 61 * SX                 # radio del área negra (~19 cm)
RING_R = 84 * SX                 # anillo blanco alrededor
RED_PX = (271, 233, 529, 319.5)  # mitad roja del tablero (la otra mitad morada)
PURPLE_PX = (271, 319.5, 529, 404)
STARTS_PX = {"red": (13, 328, 88, 403), "blue": (711, 328, 786, 403)}


class ManualArena:
    """Misma interfaz que arena.Arena (lo que usa physics.Sim) más el estado de las rampas."""

    kind = "manual"

    def __init__(self, seed=0):
        import random
        self.rng = random.Random(seed)
        self.line_w = 2.0
        self.Z = dict(Z)
        self.levels = {"1a": (0.0, 0.0, X_L2, H_CM), "2": (X_L2, 0.0, X_R2, H_CM),
                       "1b": (X_R2, 0.0, W_CM, H_CM)}
        self.stairs_l = (X_L2 - STAIR_RUN, px(0, 158)[1], X_L2, px(0, 83)[1])
        self.stairs_r = (X_R2, px(0, 158)[1], X_R2 + STAIR_RUN, px(0, 83)[1])
        self.flights = _flight_rects(-1) + _flight_rects(1)
        self.ramps = [dict(r, active=False) for r in RAMPS]
        self.obstacles = []
        self.tope = (-100.0, -100.0, -99.0, -99.0)
        self.card_colors = []
        self.holes = []
        self.props = {}
        for k, (x, y) in enumerate(PEDESTALS_PX):
            cx, cy = px(x, y)
            h = PEDESTAL / 2
            self.props["pedestal%d" % (k + 1)] = ((cx - h, cy - h, cx + h, cy + h), PEDESTAL_H, "fixed")
        self.bar = rect_px(*BAR_PX)
        self.props["bar"] = (self.bar, BAR_H, "fixed")
        yb = px(0, (BAR_PX[1] + BAR_PX[3]) / 2)[1]
        self.slots = [(px(x, 0)[0], yb) for x in SLOTS_PX]
        self.disc = px(*DISC_PX)
        self.disc_r = DISC_R
        self.ring_r = RING_R
        self.red = rect_px(*RED_PX)
        self.purple = rect_px(*PURPLE_PX)
        self.starts = {k: rect_px(*r) for k, r in STARTS_PX.items()}
        self.cylinders0 = [px(*c) for c in CYLINDERS_PX]
        self.walls = []
        self._update_walls()

    # ------------------------------------------------------------ rampas
    def _update_walls(self):
        """Las rampas levantadas son una pared en la orilla del nivel 2."""
        self.walls = []
        for r in self.ramps:
            if not r["active"]:
                x = r["hinge_x"]
                self.walls.append(((x, r["rect"][1]), (x, r["rect"][3])))

    def activate(self, i):
        self.ramps[i]["active"] = True
        self._update_walls()

    # ------------------------------------------------------------ consultas
    def material(self, x, y):
        return 0

    def height(self, x, y):
        if not (0 <= x <= W_CM and 0 <= y <= H_CM):
            return None
        for r in self.ramps:
            if r["active"] and ar.inside(r["rect"], x, y):
                q = r["rect"]
                f = (x - q[0]) / (q[2] - q[0]) if r["out"] < 0 else (q[2] - x) / (q[2] - q[0])
                return Z[1] + f * (Z[2] - Z[1])
        for st, up in ((self.stairs_l, 1), (self.stairs_r, -1)):
            if ar.inside(st, x, y):
                f = (x - st[0]) / (st[2] - st[0]) if up > 0 else (st[2] - x) / (st[2] - st[0])
                return Z[1] + f * (Z[2] - Z[1])
        if X_L2 <= x <= X_R2:
            return Z[2]
        return Z[1]

    def surface(self, x, y):
        if not (0 <= x <= W_CM and 0 <= y <= H_CM):
            return None
        for k, r in enumerate(self.ramps):
            if r["active"] and ar.inside(r["rect"], x, y):
                return "ramp%d" % (k + 1)
        if ar.inside(self.stairs_l, x, y) or ar.inside(self.stairs_r, x, y):
            return "stairs"
        return 2 if X_L2 <= x <= X_R2 else 1

    def zone_at(self, x, y):
        for k, r in self.starts.items():
            if ar.inside(r, x, y):
                return "Inicio %s" % k
        return None

    def start_pose(self, level):
        r = self.starts["blue" if level % 2 == 0 else "red"]
        return (r[0] + r[2]) / 2, (r[1] + r[3]) / 2, 90.0

    def in_disc(self, x, y, r=0.0):
        return math.hypot(x - self.disc[0], y - self.disc[1]) + r <= self.disc_r

    def to_json(self):
        return {
            "kind": "manual", "w": W_CM, "h": H_CM, "line_w": self.line_w,
            "levels": self.levels, "z": {str(k): v for k, v in self.Z.items()},
            "stairs": [self.stairs_l, self.stairs_r], "flights": self.flights,
            "stair_steps": STAIR_STEPS,
            "ramps": [{"rect": r["rect"], "out": r["out"], "hinge_x": r["hinge_x"],
                       "name": r["name"]} for r in self.ramps],
            "ramp_up_h": RAMP_UP_H,
            "props": {k: {"rect": r, "h": h, "base": self.height((r[0] + r[2]) / 2, (r[1] + r[3]) / 2)}
                      for k, (r, h, _) in self.props.items()},
            "slots": self.slots, "disc": self.disc, "disc_r": self.disc_r, "ring_r": self.ring_r,
            "red": self.red, "purple": self.purple, "starts": self.starts,
            "cube": CUBE, "cyl_d": CYL_D, "cyl_h": CYL_H, "prism_h": 15.0,
            # para el código común del visor (pista automática)
            "zones": [], "lines": [], "walls": [], "obstacles": [],
        }
