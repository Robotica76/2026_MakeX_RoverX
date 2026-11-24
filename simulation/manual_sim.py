"""Física de la ARENA MANUAL (misiones de alianza) sobre physics.Sim.

Objetos sueltos (cubos M-A-K-E-X y cilindros de 5 x 15 cm) con una dinámica sencilla pero creíble:
- El robot (chasis y garra) los EMPUJA: se deslizan alejándose del punto de contacto, el cubo gira
  si se empuja de lado, y empujan a otros objetos. Contra un pedestal, la barra o una rampa
  levantada se atoran y frenan al robot.
- Un cilindro parado se VOLTEA si lo golpean arriba de ~60 % de su altura (la garra alta) o rápido
  (> 28 cm/s); al caer de un nivel o al soltarlo desde más de 2.5 cm también se voltea.
- Gravedad: si lo que tienen debajo desaparece (orilla de un nivel, pedestal) caen a lo de abajo.
- La garra los toma solo si están entre los dedos, a la altura correcta y la garra cierra lo
  suficiente (separación de los dedos < ancho del objeto). Abrirla los suelta: caen sobre lo que
  haya debajo (piso, pedestal, lugar de la barra).
- Un cilindro que se carga y toca el piso se resbala hacia arriba entre los dedos.

Rampas: empiezan levantadas (pared en la orilla del nivel 2). Desde el nivel 2, empujarlas hacia
afuera las tumba: quedan como rampa.
"""

import math

import arena as ar
import manual_arena as ma
import physics

FRONT = -90.0                   # "de frente" = la letra mira hacia -y (hacia los equipos)
BODY_H = 12.0                   # alto del chasis que empuja (cm)
BODY_PUSH_H = 6.0               # altura a la que empuja el frente (orugas): no voltea un cilindro
TIP_SPEED = 28.0                # cm/s: un golpe más rápido voltea un cilindro parado
TIP_FRAC = 0.6                  # contacto arriba de esta fracción de la altura -> se voltea
PALM_SLIDE_SPEED = 30.0         # cm/s: más rápido, la palma tira el objeto del pedestal
GRIP_MIN_GAP = 4.6              # separación de los dedos con la garra en claw_cube (cm)


def _norm(a):
    return (a + 180.0) % 360.0 - 180.0


class ManualSim(physics.Sim):
    def _initial_pose(self, level):
        return self.arena.start_pose(level)

    def _init_objects(self, level):
        a = self.arena
        letters = list(ma.LETTERS)
        self.rng.shuffle(letters)
        self.objects = {}
        for k, L in enumerate(letters):
            r = a.props["pedestal%d" % (k + 1)][0]
            cx, cy = (r[0] + r[2]) / 2, (r[1] + r[3]) / 2
            top = a.height(cx, cy) + ma.PEDESTAL_H
            self.objects[L] = {"kind": "cube", "x": cx, "y": cy, "z": top + ma.CUBE / 2, "yaw": 0.0,
                               "face": self.rng.choice((0.0, 90.0, 180.0, -90.0)),
                               "upright": True, "held": False, "on": "pedestal%d" % (k + 1)}
        for k, (cx, cy) in enumerate(a.cylinders0):
            self.objects["c%d" % (k + 1)] = {"kind": "cyl", "x": cx, "y": cy,
                                             "z": a.height(cx, cy) + ma.CYL_H / 2, "yaw": 0.0,
                                             "face": 0.0, "upright": True, "held": False,
                                             "on": "piso"}
        self.prism = {"x": -50.0, "y": -50.0, "half": 3.2, "lost": True}
        self._gap_prev = None
        self._stuck_logged = -9.0

    # ------------------------------------------------------------ forma de los objetos
    @staticmethod
    def half_h(o):
        if o["kind"] == "cube":
            return ma.CUBE / 2
        return ma.CYL_H / 2 if o["upright"] else ma.CYL_D / 2

    @staticmethod
    def width(o):
        return ma.CUBE if o["kind"] == "cube" else ma.CYL_D

    @staticmethod
    def poly(o, x=None, y=None, yaw=None):
        x = o["x"] if x is None else x
        y = o["y"] if y is None else y
        yaw = o["yaw"] if yaw is None else yaw
        if o["kind"] == "cube":
            return ar.rect_poly(x, y, math.radians(yaw), ma.CUBE, ma.CUBE)
        if o["upright"]:
            r = ma.CYL_D / 2
            return [(x + r * math.cos(k * math.pi / 4), y + r * math.sin(k * math.pi / 4)) for k in range(8)]
        return ar.rect_poly(x, y, math.radians(yaw), ma.CYL_H, ma.CYL_D)

    def _name(self, n):
        return ("el cubo %s" % n) if self.objects[n]["kind"] == "cube" else ("el cilindro %s" % n[1:])

    def support(self, x, y):
        """Altura de lo que hay debajo de (x, y): piso, pedestal o barra (None = fuera)."""
        a = self.arena
        z = a.height(x, y)
        if z is None:
            return None
        for name, (r, h, _) in a.props.items():
            if ar.inside(r, x, y):
                z = max(z, a.height((r[0] + r[2]) / 2, (r[1] + r[3]) / 2) + h)
        return z

    # ------------------------------------------------------------ garra
    def gap(self):
        """Separación entre el dedo y la placa (cm): claw_open_w abierta, GRIP_MIN_GAP en claw_cube."""
        f = self.claw_open_frac()
        return GRIP_MIN_GAP + f * (self.body.get("claw_open_w", 9.0) - GRIP_MIN_GAP)

    def claw_z(self):
        return self.claw_point()[2]

    def update_gripper(self):
        g = self.gap()
        prev = self._gap_prev if self._gap_prev is not None else g
        self._gap_prev = g
        if self.held is None and g < prev:          # cerrando
            cx, cy, cz = self.claw_point()
            r = self.claw_reach()
            lat = self.body.get("arm_lat", 0.0)
            for name, o in self.objects.items():
                if o["held"] or o["on"] == "fuera":
                    continue
                w = self.width(o)
                if g > w - 0.2 or prev <= w - 0.2:    # aprieta justo en este paso (no antes ni después)
                    continue
                if o["kind"] == "cyl" and not o["upright"]:
                    continue
                f, l = self._local(o["x"], o["y"])
                # el dedo barre el objeto contra la placa fija: basta con que estuviera entre ellos
                side = max(0.6, (self.body.get("claw_open_w", 9.0) - w) / 2.0 + 0.3)
                # entre la palma y casi la punta de los dedos (claw_tip); más afuera se resbala
                if not (-1.0 <= f - r <= self.body["claw_tip"] - 0.3) or abs(l - lat) > side:
                    continue
                hh = self.half_h(o)
                if o["kind"] == "cube":
                    if abs(o["z"] - cz) > 2.0:
                        continue
                elif not (o["z"] - hh + 1.5 <= cz <= o["z"] + hh - 1.0):
                    continue
                self.held = name
                o.update(held=True, on=None, dz=o["z"] - cz, df=f - r, dyaw=_norm(o["yaw"] - self.yaw),
                         dface=_norm(o["face"] - self.yaw))
                self.log.append((self.world.t, "toma %s" % self._name(name)))
                break
        elif self.held is not None and g > self.width(self.objects[self.held]) + 0.3:
            self._release()
        if self.held:
            self._carry()

    def _carry(self):
        o = self.objects[self.held]
        cx, cy, cz = self.claw_point()
        c, s_ = self.frame()
        df = o.get("df", 0.0) * math.cos(math.radians(self.pitch))
        o["x"], o["y"] = cx + c * df, cy + s_ * df
        o["yaw"] = self.yaw + o.get("dyaw", 0.0)
        o["face"] = self.yaw + o.get("dface", 0.0)
        o["z"] = cz + o.get("dz", 0.0)
        s = self.support(o["x"], o["y"])
        bottom = o["z"] - self.half_h(o)
        if s is not None and bottom < s:
            # toca el piso: se resbala hacia arriba entre los dedos (no atraviesa el piso)
            o["dz"] = o.get("dz", 0.0) + (s - bottom)
            o["z"] = s + self.half_h(o)
            if self.world.t - self._hit_logged.get("objeto-piso", -9) > 1.0:
                self.log.append((self.world.t, "%s toca el piso y se resbala en la garra"
                                 % self._name(self.held)))
            self._hit_logged["objeto-piso"] = self.world.t

    def _release(self):
        name = self.held
        o = self.objects[name]
        self.held = None
        o["held"] = False
        a = self.arena
        where = None
        bottom = o["z"] - self.half_h(o)
        if o["kind"] == "cube":
            bar = a.props["bar"]
            top = (a.height((bar[0][0] + bar[0][2]) / 2, (bar[0][1] + bar[0][3]) / 2) or 0.0) + bar[1]
            for k, (sx, sy) in enumerate(a.slots):
                if math.hypot(o["x"] - sx, o["y"] - sy) <= 2.2 and bottom >= top - 1.5:
                    busy = any(p is not o and p.get("on") == "barra%d" % (k + 1) for p in self.objects.values())
                    if busy:
                        continue
                    snap = round(o["yaw"] / 90.0) * 90.0 - o["yaw"]
                    o.update(x=sx, y=sy, z=top + ma.CUBE / 2, yaw=o["yaw"] + snap, face=o["face"] + snap)
                    where = "barra%d" % (k + 1)
                    break
        if where is None:
            s = self.support(o["x"], o["y"])
            if s is None:
                where = "fuera"
                o["z"] = -10.0
            else:
                drop = bottom - s
                if o["kind"] == "cyl" and drop > 2.5:
                    self._tip(o, self.yaw + self.rng.uniform(-60, 60), "se cae al soltarlo")
                    s = self.support(o["x"], o["y"])
                    s = 0.0 if s is None else s
                o["z"] = s + self.half_h(o)
                where = self._where(o)
        o["on"] = where
        self.log.append((self.world.t, "suelta %s en %s" % (self._name(name), where)))

    def _where(self, o):
        a = self.arena
        for pname, (r, h, _) in a.props.items():
            if ar.inside(r, o["x"], o["y"]):
                return pname
        lvl = a.surface(o["x"], o["y"])
        return "nivel %s" % lvl if isinstance(lvl, int) else str(lvl)

    def _tip(self, o, direction, why):
        """Voltea un cilindro parado hacia `direction` (grados): queda acostado."""
        if o["kind"] != "cyl" or not o["upright"]:
            return
        d = math.radians(direction)
        o["upright"] = False
        o["yaw"] = direction
        base = o["z"] - ma.CYL_H / 2
        o["x"] += math.cos(d) * (ma.CYL_H - ma.CYL_D) / 2
        o["y"] += math.sin(d) * (ma.CYL_H - ma.CYL_D) / 2
        s = self.support(o["x"], o["y"])
        o["z"] = (base if s is None else s) + ma.CYL_D / 2
        self.log.append((self.world.t, "%s %s" % (self._name([k for k, v in self.objects.items() if v is o][0]), why)))

    # ------------------------------------------------------------ empujar
    def palm_poly(self, x, y, yaw):
        """Fondo de la garra (lo que empuja a un objeto que está entre los dedos abiertos)."""
        r = self.claw_reach() - 2.5 - 0.5
        lat = self.body.get("arm_lat", 0.0)
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        return ar.rect_poly(x + c * r - s * lat, y + s * r + c * lat, math.radians(yaw), 1.0, 6.0)

    @staticmethod
    def _local_at(x, y, rx, ry, ryaw):
        c, s = math.cos(math.radians(ryaw)), math.sin(math.radians(ryaw))
        dx, dy = x - rx, y - ry
        return dx * c + dy * s, -dx * s + dy * c

    def _obstacle(self, o, poly):
        """¿El objeto en esa forma choca con algo fijo (pedestal, barra, rampa levantada, pared de
        un nivel más alto)?"""
        a = self.arena
        lo, hi = o["z"] - self.half_h(o), o["z"] + self.half_h(o)
        for name, (r, h, _) in a.props.items():
            base = a.height((r[0] + r[2]) / 2, (r[1] + r[3]) / 2) or 0.0
            if lo < base + h - 0.3 and hi > base and ar.polys_overlap(poly, ar.rect_to_poly(r)):
                return name
        for p, q in a.walls:
            if hi > ma.Z[2] - 0.5 and lo < ma.Z[2] + ma.RAMP_UP_H and ar.poly_seg_overlap(poly, p, q):
                return "rampa levantada"
        for qx, qy in poly:
            z = a.height(qx, qy)
            if z is not None and z > lo + 1.0:
                return "pared del nivel"
        return None

    def _push(self, rx, ry, ryaw, commit):
        """Empuja los objetos que toca el robot en la pose (rx, ry, ryaw). Devuelve False si alguno
        queda atorado (el robot no puede avanzar)."""
        fp = self.footprint(rx, ry, ryaw)
        cp = self.claw_poly(rx, ry, ryaw)
        reach = self.claw_reach()
        lat = self.body.get("arm_lat", 0.0)
        gap = self.gap()
        palm = self.palm_poly(rx, ry, ryaw)
        rz = self.arena.height(rx, ry)
        rz = self.z if rz is None else rz
        h, _ = self.arm_h()
        cz = rz + h
        v = (self.vl + self.vr) / 2.0
        if abs(self.vl) + abs(self.vr) < 0.5:       # quieto: no empuja (soltar en la barra, etc.)
            return True
        c, s = math.cos(math.radians(ryaw)), math.sin(math.radians(ryaw))
        hx, hy = (c, s) if v >= 0 else (-c, -s)
        state = {}
        ok = True
        for name, o in self.objects.items():
            if o["held"] or o["on"] == "fuera":
                continue
            hh = self.half_h(o)
            lo, hi = o["z"] - hh, o["z"] + hh
            P = self.poly(o)
            by_body = lo < rz + BODY_H and hi > rz + 0.5 and ar.polys_overlap(fp, P)
            # entre los dedos abiertos el objeto no se empuja: solo la palma (el fondo de la garra)
            cpo = cp
            if gap > self.width(o) - 0.2:               # el dedo todavía no lo aprieta
                f, l = self._local_at(o["x"], o["y"], rx, ry, ryaw)
                if abs(l - lat) <= (self.body.get("claw_open_w", 9.0) - self.width(o)) / 2.0 + 0.3:
                    cpo = palm
            by_claw = lo < cz + 3.0 and hi > cz - 3.0 and ar.polys_overlap(cpo, P)
            cp_hit = cpo
            if not (by_body or by_claw):
                continue
            # un objeto sobre un pedestal o en la barra tiene fricción: la palma lo TOCA y el robot
            # se detiene ahí (listo para cerrar la garra); solo un golpe rápido lo tira
            if (by_claw and not by_body and cpo is palm and abs(v) < PALM_SLIDE_SPEED
                    and str(o.get("on", "")).startswith(("pedestal", "barra"))):
                ok = False
                if commit is False and self.world.t - self._stuck_logged > 1.5:
                    self._stuck_logged = self.world.t
                    self.log.append((self.world.t, "la garra toca %s: ciérrala para tomarlo"
                                     % self._name(name)))
                continue
            dx, dy = o["x"] - rx, o["y"] - ry
            n = math.hypot(dx, dy) or 1.0
            ux, uy = dx / n, dy / n
            px_, py_ = 0.6 * hx + 0.4 * ux, 0.6 * hy + 0.4 * uy
            m = math.hypot(px_, py_) or 1.0
            px_, py_ = px_ / m, py_ / m
            x, y, yaw = o["x"], o["y"], o["yaw"]
            moved = 0.0
            for _ in range(48):
                Q = self.poly(o, x, y, yaw)
                if not ((by_body and ar.polys_overlap(fp, Q)) or (by_claw and ar.polys_overlap(cp_hit, Q))):
                    break
                x += px_ * 0.25
                y += py_ * 0.25
                moved += 0.25
                if o["kind"] == "cube" or not o["upright"]:
                    yaw += 6.0 * (hx * uy - hy * ux)          # empujado de lado: gira
            if moved == 0.0:
                continue
            hit = self._obstacle(o, self.poly(o, x, y, yaw))
            if hit:
                ok = False
                if commit is False and self.world.t - self._stuck_logged > 1.0:
                    self._stuck_logged = self.world.t
                    self.log.append((self.world.t, "%s se atora contra %s" % (self._name(name), hit)))
                continue
            state[name] = (x, y, yaw, by_claw, moved)
        if not commit:
            return ok
        for name, (x, y, yaw, by_claw, moved) in state.items():
            o = self.objects[name]
            dyaw = yaw - o["yaw"]
            o["x"], o["y"], o["yaw"] = x, y, yaw
            if o["kind"] == "cube":
                o["face"] += dyaw
            if not o.get("pushed_log") or self.world.t - o["pushed_log"] > 2.0:
                self.log.append((self.world.t, "empuja %s" % self._name(name)))
            o["pushed_log"] = self.world.t
            if o["kind"] == "cyl" and o["upright"]:
                contact = (cz if by_claw else rz + BODY_PUSH_H) - (o["z"] - ma.CYL_H / 2)
                if contact > TIP_FRAC * ma.CYL_H or abs(v) > TIP_SPEED:
                    self._tip(o, math.degrees(math.atan2(hy, hx)), "se voltea (golpe %s)" %
                              ("alto" if contact > TIP_FRAC * ma.CYL_H else "rápido"))
            o["on"] = self._where(o)
        self._separate()
        return ok

    def _separate(self):
        """Objetos encimados se separan (uno empuja al otro)."""
        names = [n for n, o in self.objects.items() if not o["held"] and o["on"] != "fuera"]
        for _ in range(3):
            moved = False
            for i, a_ in enumerate(names):
                for b_ in names[i + 1:]:
                    A, B = self.objects[a_], self.objects[b_]
                    if abs(A["z"] - B["z"]) > self.half_h(A) + self.half_h(B):
                        continue
                    if not ar.polys_overlap(self.poly(A), self.poly(B)):
                        continue
                    dx, dy = B["x"] - A["x"], B["y"] - A["y"]
                    n = math.hypot(dx, dy) or 1.0
                    for _k in range(16):
                        nx, ny = B["x"] + dx / n * 0.25, B["y"] + dy / n * 0.25
                        if self._obstacle(B, self.poly(B, nx, ny)):
                            break
                        B["x"], B["y"] = nx, ny
                        moved = True
                        if not ar.polys_overlap(self.poly(A), self.poly(B)):
                            break
            if not moved:
                break

    def _gravity(self):
        for name, o in self.objects.items():
            if o["held"] or o["on"] == "fuera":
                continue
            s = self.support(o["x"], o["y"])
            if s is None:
                o["on"] = "fuera"
                o["z"] = -10.0
                self.log.append((self.world.t, "%s se sale de la arena" % self._name(name)))
                continue
            bottom = o["z"] - self.half_h(o)
            if bottom > s + 0.3:
                drop = bottom - s
                if o["kind"] == "cyl" and o["upright"] and drop > 2.5:
                    self._tip(o, o["yaw"] if o["yaw"] else self.rng.uniform(-180, 180), "se cae y se voltea")
                    s = self.support(o["x"], o["y"])
                    s = 0.0 if s is None else s
                o["z"] = s + self.half_h(o)
                o["on"] = self._where(o)
                self.log.append((self.world.t, "%s cae %.0f cm (%s)" % (self._name(name), drop, o["on"])))

    # ------------------------------------------------------------ rampas y choques
    def _ramp_push(self, x, y, yaw):
        a = self.arena
        if self.z < ma.Z[2] - 3.0:
            return
        fp = self.footprint(x, y, yaw)
        cp = self.claw_poly(x, y, yaw)
        v = (self.vl + self.vr) / 2.0
        for i, r in enumerate(a.ramps):
            if r["active"]:
                continue
            p, q = (r["hinge_x"], r["rect"][1]), (r["hinge_x"], r["rect"][3])
            if not (ar.poly_seg_overlap(fp, p, q) or ar.poly_seg_overlap(cp, p, q)):
                continue
            toward = math.cos(math.radians(yaw)) * r["out"] * (1 if v >= 0 else -1)
            if toward > 0.3 and abs(v) > 1.0:
                a.activate(i)
                self.ramp_t = getattr(self, "ramp_t", {})
                self.ramp_t[i] = self.world.t
                self.log.append((self.world.t, "activa la rampa %s (cae hacia afuera)" % r["name"]))

    def _blocked(self, x, y, yaw):
        self._ramp_push(x, y, yaw)
        if super()._blocked(x, y, yaw):
            return True
        return not self._push(x, y, yaw, commit=False)

    def _objects_step(self):
        self._push(self.x, self.y, self.yaw, commit=True)
        self._gravity()

    def place(self, level, noise=None):
        x, y, yaw = self.arena.start_pose(level)
        n = self.noise if noise is None else noise
        self.x = x + self.rng.gauss(0, 0.7) * n
        self.y = y + self.rng.gauss(0, 0.7) * n
        self.yaw = yaw + self.rng.gauss(0, 1.5) * n
        self._last_yaw = self.yaw
        self.vl = self.vr = 0.0
        self.acc = 0.0
        self.z = self.arena.height(self.x, self.y)
        self._update_pitch()
        self.fallen = None
        self.contacts = set()
        if self.held:
            self._carry()

    # ------------------------------------------------------------ cuadros para el visor
    def frame_extra(self):
        objs = []
        for name, o in self.objects.items():
            objs.append([name, o["kind"], round(o["x"], 1), round(o["y"], 1), round(o["z"], 1),
                         round(o["yaw"], 0), 1 if o["upright"] else 0, 1 if o["held"] else 0,
                         round(_norm(o["face"]), 0)])
        return [objs, [1 if r["active"] else 0 for r in self.arena.ramps]]


MISSIONS = [
    {"id": "A1", "name": "MakeX: letras en la barra (50 c/u)", "pts": 250, "level": 2},
    {"id": "A1+", "name": "MakeX en orden y de frente (bono)", "pts": 100, "level": 2},
    {"id": "A2", "name": "Rampas activadas (30 c/u)", "pts": 60, "level": 2},
    {"id": "A2+", "name": "Las dos rampas (bono)", "pts": 40, "level": 2},
    {"id": "A3", "name": "Cilindros parados en el área negra (50 c/u)", "pts": 200, "level": 2},
]


class ManualReferee:
    """Puntos de las misiones de alianza (estado actual de la arena)."""

    def __init__(self, sim):
        self.sim = sim
        self.done = {}
        self.first = {}

    def score(self):
        s = self.sim
        a = s.arena
        on_bar = {}
        for name, o in s.objects.items():
            if o["kind"] == "cube" and not o["held"] and str(o.get("on", "")).startswith("barra"):
                on_bar[int(o["on"][5:]) - 1] = (name, o)
        letters = len(on_bar)
        word = (letters == 5 and all(on_bar[k][0] == ma.LETTERS[k] for k in range(5))
                and all(abs(_norm(on_bar[k][1]["face"] - FRONT)) < 45 for k in range(5)))
        ramps = sum(1 for r in a.ramps if r["active"])
        cyl = 0
        for o in s.objects.values():
            if o["kind"] == "cyl" and o["upright"] and not o["held"] and \
                    abs(o["z"] - ma.CYL_H / 2 - ma.Z[2]) < 0.5 and a.in_disc(o["x"], o["y"], ma.CYL_D / 2):
                cyl += 1
        return {"A1": (letters > 0, 50 * letters), "A1+": (word, 100 if word else 0),
                "A2": (ramps > 0, 30 * ramps), "A2+": (ramps == 2, 40 if ramps == 2 else 0),
                "A3": (cyl > 0, 50 * cyl)}

    def update(self):
        t = self.sim.world.t
        for k, (ok, pts) in self.score().items():
            if ok and k not in self.first:
                self.first[k] = t
            if not ok:
                self.first.pop(k, None)

    def snapshot(self):
        out = []
        for m in MISSIONS:
            ok, pts = self.score()[m["id"]]
            out.append({"id": m["id"], "ok": ok, "pts": pts, "t": self.first.get(m["id"])})
        return out
