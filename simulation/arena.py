"""Pista automática de Rover X para el simulador.

Reconstruida de la vista superior del reglamento
(resources/reglamento/arena/arena_automatica_superior.jpg, 1093 x 570 px). La imagen está girada
180° respecto a la vista isométrica, pero NO reflejada, así que izquierda y derecha se conservan.

Escala: cada tablero mide 610 mm (2440 / 4) y ocupa 260 px, así que 1 px = 0.2346 cm. Las
coordenadas se escriben en PIXELES DE LA IMAGEN para poder compararlas con ella y se convierten a
cm con px(). Marco en cm: x hacia la derecha de la imagen, y hacia ARRIBA de la imagen, origen en la
esquina inferior izquierda de la arena; rumbo en grados, 0 = +x, positivo = antihorario.

TODO(pista): todo sale de una imagen; medir en la pista real las alturas de los niveles, la
posición de los props y de las marcas, y la de las tarjetas (pregunta abierta del reglamento).
"""

import math
import random

S = 61.0 / 260.0            # cm por pixel de la imagen
X0, Y0 = 20, 545            # esquina inferior izquierda de la arena en la imagen (px)
W_CM = 1040 * S             # 244 cm
H_CM = 520 * S              # 122 cm


def px(x, y):
    """Pixel de la imagen -> cm en el marco del simulador."""
    return ((x - X0) * S, (Y0 - y) * S)


def rect_px(x0, y0, x1, y1):
    """Rectángulo en pixeles -> (xmin, ymin, xmax, ymax) en cm."""
    a = px(x0, y0)
    b = px(x1, y1)
    return (min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1]))


def inside(r, x, y, margin=0.0):
    return r[0] - margin <= x <= r[2] + margin and r[1] - margin <= y <= r[3] + margin


# ---------------------------------------------------------------- alturas (TODO medir)
Z = {1: 0.0, 2: 12.0, 3: 24.0, 4: 44.0}     # cm sobre el nivel 1 (escalera: 8 x 2.5 cm)

# ---------------------------------------------------------------- materiales del piso
# código -> (nombre del color que da get_color_sta, luz reflejada 0..100, RGB)
# TODO(calibrar): luz y RGB salen de valores típicos del Quad RGB, no de la pista real.
MATERIALS = {
    0: ("white", 88, (230, 230, 230)),     # piso
    1: ("white", 62, (160, 160, 160)),     # rampa gris del nivel 1
    2: ("black", 12, (25, 25, 25)),        # línea
    3: ("red", 58, (200, 40, 40)),
    4: ("yellow", 82, (220, 200, 50)),
    5: ("green", 35, (40, 160, 70)),
    6: ("blue", 40, (30, 120, 200)),       # zona azul del nivel 3
    7: ("blue", 26, (60, 60, 140)),        # X azul de la meta
    8: ("cyan", 60, (60, 180, 200)),
    9: ("purple", 38, (140, 60, 170)),
}
COLOR_CODE = {"red": 3, "yellow": 4, "green": 5, "blue": 6, "cyan": 8, "purple": 9}

# ---------------------------------------------------------------- líneas (px)
L1_LINES = [
    [(73.5, 76), (73.5, 263.5)],                       # vertical que sale de la X roja
    [(73.5, 78.5), (345, 78.5), (345, 121)],           # línea superior
    [(173.5, 155), (218.5, 155)],                      # línea punteada hacia la rampa
    [(244.5, 155), (278.5, 155)],
    [(301, 155), (343.5, 155)],
    [(370, 155), (592, 155)],                          # sigue por la rampa hasta el nivel 2
    [(177, 151), (177, 233.5)],
    [(42, 233.5), (348, 233.5)],                       # línea inferior (con el tope)
    [(347.5, 187.5), (347.5, 280)],                    # cruce frente al pedestal del cubo
]
L2_LINES = [
    [(545, 90), (642, 90)],                            # cruce frente al pedestal del nivel 2
    [(592, 90), (592, 197), (622, 231), (672, 231)],   # baja por la zona amarilla al pasillo
]
# salida del pasillo, rampa blanca y curva en S del nivel 3 hasta la escalera y la meta
S_CURVE = [(927, 76), (965, 76), (995, 108), (995, 300), (990, 330), (975, 345), (950, 351),
           (927, 344), (896, 342), (860, 336), (835, 350), (826, 380), (835, 407), (865, 425),
           (920, 417), (970, 411), (990, 415), (1010, 440), (1012, 465), (995, 490),
           (965, 503), (910, 505), (860, 495), (820, 472), (800, 457), (775, 435), (740, 417),
           (717, 414), (540, 414), (73, 415)]


def chaikin(pts, n=2, keep=4):
    """Suaviza una polilínea (las primeras `keep` esquinas se dejan rectas)."""
    head, pts = pts[:keep], pts[keep - 1:]
    for _ in range(n):
        out = [pts[0]]
        for a, b in zip(pts[:-1], pts[1:]):
            out.append((0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]))
            out.append((0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1]))
        out.append(pts[-1])
        pts = out
    return head[:-1] + pts


LINES_PX = L1_LINES + L2_LINES + [chaikin(S_CURVE)]

# ---------------------------------------------------------------- zonas y marcas (px)
ZONES_PX = [
    ("red", (20, 25, 126, 132), 3, "Inicio nivel 1"),
    ("yellow", (540, 102, 592, 207), 4, "Inicio nivel 2"),
    ("blue", (944, 286, 1047, 340), 6, "Inicio nivel 3"),
    ("green", (487, 362, 541, 469), 5, "Inicio nivel 4"),
    ("blue", (20, 362, 126, 467), 7, "Meta (X azul)"),
]
GREEN_MARKS_PX = [(77, 219.5, 88.5, 230), (162, 219.5, 173.5, 230),
                  (333.5, 219.5, 345, 233), (333.5, 234, 345, 248.5), (579, 159, 589, 170)]
# marcas rojas: centro (px), ángulo (grados en la imagen), junto a la línea por fuera
RED_MARKS_PX = [((981, 504.5), 25), ((757, 436), 35)]
RED_MARK_SIZE = (3.0, 2.0)          # cm
DOTTED_ZONE_PX = (912.5, 482.5, 957.5, 525)

# ---------------------------------------------------------------- superficies (px)
LEVEL_RECTS_PX = {
    1: (20, 25, 411, 285),
    2: (541, 25, 1060, 285),
    3: (541, 285, 1060, 545),
    4: (20, 285, 541, 545),
}
RAMP1_PX = (411, 25, 541, 285)          # gris, sube hacia +x (nivel 1 -> 2)
RAMP2_PX = (944, 156, 1042, 285)        # blanca, sube hacia abajo de la imagen (nivel 2 -> 3)
STAIRS_PX = (541, 363, 669, 466)        # sube hacia -x (nivel 3 -> 4)
STAIR_FLIGHTS_PX = [(541, 363, 669, 390), (541, 440, 669, 466)]   # dos tramos, 12 cm entre ellos
STAIR_STEPS = 8
STAIR_RISE = 2.5

# ---------------------------------------------------------------- props (px)
PEDESTAL_H = 10.0
PROPS_PX = {
    # nombre: (rect px, altura cm, tipo)
    "pedestal1": ((382, 219.5, 408.5, 246), PEDESTAL_H, "fixed"),
    "pedestal2": ((580, 30, 606, 55), PEDESTAL_H, "fixed"),
    "ball_ped": ((707.5, 510, 735, 536), PEDESTAL_H, "fixed"),      # arco con la pelota arriba
    "tower": ((412, 393.5, 455, 436.5), 25.0, "fixed"),
    "bar": ((151, 292.5, 408.5, 322.5), 10.0, "fixed"),
}
PRISM_PX = (896, 327.5, 926, 357.5)
PRISM_H = 15.0
CUBE_SIZE = 5.0
BALL_D = 3.0
BAR_HOLES_PX = [(173.5, 307.5), (216, 307.5), (258.5, 307.5), (301, 307.5), (343.5, 307.5),
                (386, 307.5)]
CARD_SIZE = (4.7, 7.0)                 # a lo largo de la línea x de través (cm)
CARD_COLORS = ["red", "yellow", "green", "cyan", "blue", "purple"]   # TODO: pregunta abierta
WALLS_PX = [((672, 29), (927, 29)), ((672, 29), (672, 155)), ((672, 282), (927, 282)),
            ((926, 155), (926, 282))]
OBSTACLE_ROOM_PX = (672, 29, 927, 282)  # pasillo de obstáculos (misión 5)
TOPE_PX = (237, 181, 245, 285)          # tope gris que cruza la línea inferior del nivel 1
OBSTACLE_SIZE = 7.5

# Zonas que usa el juez (px)
M1_ZONE_PX = (300, 205, 370, 262)       # "frente al pedestal": alrededor del cruce
START_POSES_PX = {                       # (x, y, rumbo en grados en el marco en cm)
    1: (73.5, 80, -90),
    2: (579, 155, 0),          # sondas sobre la T, como al llegar por la rampa
    3: (995, 312, -90),
    4: (514, 415, 180),
}


class Arena:
    """Mapa del piso (raster), alturas y props. line_w en cm (reglamento: 2 ± 1)."""

    RES = 0.25          # cm por celda del raster

    def __init__(self, line_w=2.0, seed=0, card_colors=None, n_obstacles=4, ramp_deg=None):
        """ramp_deg: inclinación de las dos rampas (el reglamento permite hasta 40°); cambia la
        altura de los niveles 2, 3 y 4 (la escalera sigue subiendo 20 cm)."""
        self.line_w = line_w
        self.Z = dict(Z)
        if ramp_deg is not None:
            run = (RAMP1_PX[2] - RAMP1_PX[0]) * S
            rise = math.tan(math.radians(ramp_deg)) * run
            self.Z[2] = self.Z[1] + rise
            self.Z[3] = self.Z[2] + rise
            self.Z[4] = self.Z[3] + STAIR_STEPS * STAIR_RISE
        self.rng = random.Random(seed)
        self.nx = int(math.ceil(W_CM / self.RES))
        self.ny = int(math.ceil(H_CM / self.RES))
        self.grid = bytearray(self.nx * self.ny)
        self.lines = [[px(*p) for p in pl] for pl in LINES_PX]
        self.zones = [(name, rect_px(*r), code, label) for name, r, code, label in ZONES_PX]
        self.green_marks = [rect_px(*r) for r in GREEN_MARKS_PX]
        self.red_marks = [(px(*c), -a) for c, a in RED_MARKS_PX]
        self.dotted = rect_px(*DOTTED_ZONE_PX)
        self.levels = {k: rect_px(*r) for k, r in LEVEL_RECTS_PX.items()}
        self.ramp1 = rect_px(*RAMP1_PX)
        self.ramp2 = rect_px(*RAMP2_PX)
        self.stairs = rect_px(*STAIRS_PX)
        self.flights = [rect_px(*r) for r in STAIR_FLIGHTS_PX]
        self.props = {k: (rect_px(*r), h, kind) for k, (r, h, kind) in PROPS_PX.items()}
        self.walls = [(px(*a), px(*b)) for a, b in WALLS_PX]
        self.room = rect_px(*OBSTACLE_ROOM_PX)
        self.tope = rect_px(*TOPE_PX)
        self.m1_zone = rect_px(*M1_ZONE_PX)
        self.holes = [px(*h) for h in BAR_HOLES_PX]
        if card_colors is None:
            card_colors = CARD_COLORS[:]
            self.rng.shuffle(card_colors)
        self.card_colors = card_colors
        self.cards = []
        for (hx, _), color in zip(BAR_HOLES_PX, card_colors):
            cx, cy = px(hx, 415)
            self.cards.append(((cx - CARD_SIZE[0] / 2, cy - CARD_SIZE[1] / 2,
                                cx + CARD_SIZE[0] / 2, cy + CARD_SIZE[1] / 2), color))
        self.obstacles = self._place_obstacles(n_obstacles)
        self._rasterize()

    # ------------------------------------------------------------ construcción
    def _place_obstacles(self, n):
        r = self.room
        out = []
        for _ in range(n):
            x = self.rng.uniform(r[0] + 8, r[2] - 8)
            y = self.rng.uniform(r[1] + 8, r[3] - 8)
            h = OBSTACLE_SIZE / 2
            out.append((x - h, y - h, x + h, y + h))
        return out

    def _cell(self, x, y):
        i = int(x / self.RES)
        j = int(y / self.RES)
        if 0 <= i < self.nx and 0 <= j < self.ny:
            return j * self.nx + i
        return -1

    def _fill_rect(self, r, code):
        i0 = max(0, int(r[0] / self.RES))
        i1 = min(self.nx - 1, int(r[2] / self.RES))
        j0 = max(0, int(r[1] / self.RES))
        j1 = min(self.ny - 1, int(r[3] / self.RES))
        for j in range(j0, j1 + 1):
            base = j * self.nx
            for i in range(i0, i1 + 1):
                self.grid[base + i] = code

    def _fill_poly(self, poly, code):
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        for j in range(max(0, int(min(ys) / self.RES)), min(self.ny, int(max(ys) / self.RES) + 1)):
            for i in range(max(0, int(min(xs) / self.RES)), min(self.nx, int(max(xs) / self.RES) + 1)):
                if point_in_poly((i + 0.5) * self.RES, (j + 0.5) * self.RES, poly):
                    self.grid[j * self.nx + i] = code

    def _stroke(self, a, b, half, code):
        x0, x1 = min(a[0], b[0]) - half, max(a[0], b[0]) + half
        y0, y1 = min(a[1], b[1]) - half, max(a[1], b[1]) + half
        for j in range(max(0, int(y0 / self.RES)), min(self.ny, int(y1 / self.RES) + 1)):
            yc = (j + 0.5) * self.RES
            for i in range(max(0, int(x0 / self.RES)), min(self.nx, int(x1 / self.RES) + 1)):
                if seg_dist((i + 0.5) * self.RES, yc, a, b) <= half:
                    self.grid[j * self.nx + i] = code

    def _rasterize(self):
        self._fill_rect(self.ramp1, 1)
        for name, r, code, _ in self.zones:
            self._fill_rect(r, code)
        for r in self.green_marks:
            self._fill_rect(r, 5)
        for poly in self.red_mark_polys():
            self._fill_poly(poly, 3)
        half = self.line_w / 2.0
        for pl in self.lines:
            for a, b in zip(pl[:-1], pl[1:]):
                self._stroke(a, b, half, 2)
        for r, color in self.cards:
            self._fill_rect(r, COLOR_CODE[color])

    def red_mark_polys(self):
        out = []
        w, h = RED_MARK_SIZE
        for (cx, cy), ang in self.red_marks:
            out.append(rect_poly(cx, cy, math.radians(ang), w, h))
        return out

    # ------------------------------------------------------------ consultas
    def material(self, x, y):
        k = self._cell(x, y)
        if k < 0:
            return 0
        return self.grid[k]

    def height(self, x, y):
        """Altura del piso (cm) en (x, y), o None fuera de la arena."""
        if not (0 <= x <= W_CM and 0 <= y <= H_CM):
            return None
        if inside(self.ramp1, x, y):
            f = (x - self.ramp1[0]) / (self.ramp1[2] - self.ramp1[0])
            return self.Z[1] + f * (self.Z[2] - self.Z[1])
        if inside(self.ramp2, x, y):
            f = (self.ramp2[3] - y) / (self.ramp2[3] - self.ramp2[1])
            return self.Z[2] + f * (self.Z[3] - self.Z[2])
        if inside(self.stairs, x, y):
            f = (self.stairs[2] - x) / (self.stairs[2] - self.stairs[0])
            return self.Z[3] + f * (self.Z[4] - self.Z[3])
        for lvl in (1, 2, 3, 4):
            if inside(self.levels[lvl], x, y):
                return self.Z[lvl]
        return None

    def surface(self, x, y):
        """Nombre de la superficie: 1..4 (nivel plano), 'ramp1', 'ramp2', 'stairs' o None."""
        if not (0 <= x <= W_CM and 0 <= y <= H_CM):
            return None
        if inside(self.ramp1, x, y):
            return "ramp1"
        if inside(self.ramp2, x, y):
            return "ramp2"
        if inside(self.stairs, x, y):
            return "stairs"
        for lvl in (1, 2, 3, 4):
            if inside(self.levels[lvl], x, y):
                return lvl
        return None

    def zone_at(self, x, y):
        for name, r, code, label in self.zones:
            if inside(r, x, y):
                return label
        return None

    def start_pose(self, level):
        x, y, yaw = START_POSES_PX[level]
        cx, cy = px(x, y)
        return cx, cy, yaw

    def to_json(self):
        """Datos de dibujo para el visor HTML (todo en cm)."""
        return {
            "w": W_CM, "h": H_CM, "line_w": self.line_w,
            "lines": self.lines,
            "zones": [{"color": n, "rect": r, "code": c, "label": l} for n, r, c, l in self.zones],
            "green_marks": self.green_marks,
            "red_marks": self.red_mark_polys(),
            "dotted": self.dotted,
            "levels": {str(k): r for k, r in self.levels.items()},
            "z": {str(k): round(v, 1) for k, v in self.Z.items()},
            "ramps": [self.ramp1, self.ramp2],
            "stairs": self.stairs, "flights": self.flights,
            "props": {k: {"rect": r, "h": h} for k, (r, h, _) in self.props.items()},
            "walls": self.walls, "room": self.room, "tope": self.tope,
            "obstacles": self.obstacles,
            "holes": self.holes,
            "cards": [{"rect": r, "color": c} for r, c in self.cards],
            "m1_zone": self.m1_zone,
        }


# ---------------------------------------------------------------- geometría 2D
def seg_dist(x, y, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(x - ax, y - ay)
    t = ((x - ax) * dx + (y - ay) * dy) / L2
    t = 0 if t < 0 else (1 if t > 1 else t)
    return math.hypot(x - (ax + t * dx), y - (ay + t * dy))


def point_in_poly(x, y, poly):
    inside_ = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            inside_ = not inside_
        j = i
    return inside_


def rect_poly(cx, cy, ang, length, width):
    """Rectángulo orientado: centro, ángulo (rad), largo (a lo largo de ang) y ancho."""
    c, s = math.cos(ang), math.sin(ang)
    hl, hw = length / 2.0, width / 2.0
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]


def rect_to_poly(r):
    return [(r[0], r[1]), (r[2], r[1]), (r[2], r[3]), (r[0], r[3])]


def polys_overlap(p, q):
    """Separación de ejes (SAT) para dos polígonos convexos."""
    for poly in (p, q):
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            nx, ny = y1 - y2, x2 - x1
            a = [nx * x + ny * y for x, y in p]
            b = [nx * x + ny * y for x, y in q]
            if max(a) < min(b) or max(b) < min(a):
                return False
    return True


def poly_seg_overlap(poly, a, b):
    return polys_overlap(poly, [a, b])
