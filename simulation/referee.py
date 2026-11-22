"""Juez del simulador: revisa cada misión con las condiciones del reglamento.

Es independiente del programa del robot: el programa cree que logró una misión cuando sus pasos
terminan bien; el juez mira dónde quedó el robot y los objetos. Las dos columnas se comparan en
el reporte.

Interpretaciones (TODO confirmar con el reglamento o en la junta de capitanes):
- Misiones 2 y 8, "al menos a 5 cm de distancia de la base": la base del objeto queda >= 5 cm
  arriba de la cara superior del pedestal.
- "Completamente dentro" de un nivel: las cuatro esquinas de las orugas sobre la parte plana.
- Misión 11: cuenta los colores de tarjeta distintos que el robot mostró en la pantalla.
"""

import arena as ar

LIFT_CM = 5.0
HOLD_S = 5.0


class Referee:
    def __init__(self, sim, missions, start_level=1):
        self.sim = sim
        # al reiniciar en un nivel, las misiones de niveles anteriores no se juegan
        self.pts = {m["id"]: m["pts"] for m in missions if m["level"] >= start_level}
        self.done = {}              # id -> tiempo en que se logró
        self.hold = {"cube": None, "ball": None}
        self.in_room = False
        self.shown = []             # colores mostrados en pantalla (línea 3)
        self._last_label = None
        self.touched_tower = False

    # ------------------------------------------------------------ ayudas
    def _corners_on(self, level):
        s = self.sim
        return all(s.arena.surface(x, y) == level for x, y in s.footprint())

    def _latch(self, mid):
        if mid in self.pts and mid not in self.done:
            self.done[mid] = self.sim.world.t
            self.sim.log.append((self.sim.world.t, "JUEZ: mision %d lograda" % mid))

    def _stopped(self):
        return abs(self.sim.vl) < 0.5 and abs(self.sim.vr) < 0.5

    def _lifted(self, name, ped):
        s = self.sim
        o = s.objects[name]
        if not o["held"]:
            return False
        r = s.arena.props[ped][0]
        top = (s.arena.height((r[0] + r[2]) / 2, (r[1] + r[3]) / 2) or 0) + ar.PEDESTAL_H
        return o["z"] - o["size"] / 2 >= top + LIFT_CM

    # ------------------------------------------------------------ cada cuadro
    def update(self):
        s = self.sim
        t = s.world.t
        a = s.arena
        if self._stopped() and ar.inside(a.m1_zone, s.x, s.y):
            self._latch(1)
        for mid, name, ped in ((2, "cube", "pedestal1"), (8, "ball", "ball_ped")):
            if self._lifted(name, ped):
                if self.hold[name] is None:
                    self.hold[name] = t
                elif t - self.hold[name] >= HOLD_S:
                    self._latch(mid)
            else:
                self.hold[name] = None
        for mid, lvl in ((3, 2), (6, 3), (9, 4)):
            # colocado ahí por un reinicio no cuenta: hay que haberlo subido
            if mid not in self.done and lvl > s.placed_level and self._corners_on(lvl):
                self._latch(mid)
        if ar.inside(a.room, s.x, s.y):
            self.in_room = True
        elif self.in_room and s.x > a.room[2]:
            fp = s.footprint()
            if all(not ar.inside(a.room, x, y) for x, y in fp):
                self._latch(5)
        tower = a.props["tower"][0]
        if a.surface(s.x, s.y) == 4 and all(x < tower[0] for x, _ in s.footprint()):
            self._latch(10)
        if "tower" in s.contacts:
            self.touched_tower = True
        lab = s.world.labels.get(3)
        if lab and lab != self._last_label:
            self._last_label = lab
            self.shown.append(lab.strip().lower())

    # ------------------------------------------------------------ al terminar
    def final(self):
        s = self.sim
        a = s.arena
        if s.objects["cube"]["on"] == "pedestal2" and not s.objects["cube"]["held"]:
            self._latch(4)
        pr = s.prism
        if not pr["lost"]:
            h = pr["half"]
            d = a.dotted
            if pr["x"] + h > d[0] and pr["x"] - h < d[2] and pr["y"] + h > d[1] and pr["y"] - h < d[3]:
                self._latch(7)
        red = [i for i, (_, c) in enumerate(a.cards) if c == "red"]
        if red and s.objects["ball"]["on"] == "hole%d" % red[0]:
            self._latch(12)
        goal = a.zones[4][1]
        if self._stopped() and not s.fallen and all(ar.inside(goal, x, y) for x, y in s.footprint()):
            self._latch(13)
        cards = set(a.card_colors)
        seen = sorted(set(c for c in self.shown if c in cards))
        out = []
        total = 0
        falls = getattr(s, "falls", 0) + (1 if s.fallen else 0)
        penalty = 20 * max(0, falls - 1)        # la primera falta es advertencia
        for mid in sorted(self.pts):
            if mid == 11:
                p = 20 * len(seen)
                ok = len(seen) > 0
            else:
                ok = mid in self.done
                p = self.pts[mid] if ok else 0
            total += p
            out.append({"id": mid, "ok": ok, "pts": p, "t": self.done.get(mid)})
        notes = []
        if self.touched_tower:
            notes.append("tocó la torre")
        if s.fallen:
            notes.append("CAÍDA: " + s.fallen)
        notes_tip = s.min_tip
        if s.slides:
            notes.append("se resbaló hacia atrás en una pendiente %d vez/veces" % s.slides)
        if falls:
            notes.append("caídas: %d (penalización %d)" % (falls, penalty))
        restarts = getattr(s, "restarts", [])
        if restarts:
            notes.append("reinicios: %s" % ", ".join("nivel %d" % lv for lv in restarts))
        return {"missions": out, "total": total - penalty, "cards_seen": seen, "notes": notes,
                "min_tip_cm": round(notes_tip, 1), "restarts": len(restarts), "falls": falls}
