"""Física del simulador: el robot (orugas, sensores, brazo y garra) y los objetos de la pista.

El programa REAL del robot (software/) corre sin cambios sobre los módulos falsos de
tests/fakes (cyberpi, mbot2, mbuild, gamepad), que leen y escriben en `world`. Esta clase
reemplaza el mundo mínimo de las pruebas por uno físico:

- hal.wait() -> time.sleep() -> Sim.sleep(): avanza la física en pasos de DT y el reloj virtual.
- Motores: la potencia de cada oruga da una velocidad (con zona muerta, retardo y la carga de la
  pendiente). El avance real usa `slip`; los encoders miden la rueda, no el piso.
- Giroscopio: rumbo real (+ deriva y ruido opcionales). Pitch: alturas del piso bajo la punta y
  la cola de las orugas.
- Quad RGB: cada sonda promedia un círculo de piso (raster de la pista) -> luz, color y RGB.
- Brazo y garra: la altura sale del ángulo del servo con la misma geometría del robot.
- Choques con props fijos y muros: el movimiento se rechaza. Caídas: fin de la simulación.

La geometría del robot "verdadero" es una copia de cfg.P al empezar (`true_overrides` la cambia
para probar qué pasa si una medida está mal).
"""

import math
import random
import time

import arena as ar

DT = 0.005                  # paso de la física (s)
FRAME_S = 0.05              # cada cuánto se graba un cuadro y revisa el juez


class Fallen(Exception):
    """El robot se cayó de un nivel o de la arena (termina la corrida)."""


class Sim:
    def __init__(self, world, cfg, geo, arena, level=1, noise=0.0, seed=0, true_overrides=None,
                 log=None):
        self.world = world
        self.cfg = cfg
        self.geo = geo
        self.arena = arena
        self.rng = random.Random(seed)
        self.noise = noise
        self.log = log if log is not None else []
        self.body = dict(cfg.P)
        if true_overrides:
            self.body.update(true_overrides)
        n = noise
        # parámetros físicos "verdaderos" (con ruido se sortean por corrida)
        self.vmax = 46.0 * (1 + n * self.rng.uniform(-0.08, 0.08))    # cm/s al 100 %
        self.deadband = 4.0
        self.tau = 0.08
        self.slip_true = 0.97 * (1 + n * self.rng.uniform(-0.03, 0.03))
        self.skid = 1.3 * (1 + n * self.rng.uniform(-0.1, 0.1))       # orugas derrapan al girar
        self.bias = 1 + n * self.rng.uniform(-0.03, 0.03)              # una oruga más rápida
        self.gyro_drift = n * self.rng.uniform(-0.3, 0.3)              # grados/min
        self.light_sigma = 2.0 * n
        self.ambient = 1 + n * self.rng.uniform(-0.08, 0.08)
        self.slope_k = 30.0                                            # cm/s perdidos por sen(pitch)
        self.spot = 0.35                                               # radio de lectura de una sonda (cm)

        x, y, yaw = arena.start_pose(level)
        self.x, self.y, self.yaw = x, y, yaw
        self.vl = self.vr = 0.0
        self.z = arena.height(x, y)
        self.pitch = 0.0
        self.t_phys = 0.0
        self.acc = 0.0                  # aceleración longitudinal filtrada (cm/s²)
        self.min_tip = 99.0             # menor margen contra volcadura de la corrida (cm)
        self.contacts = set()           # props tocados ahora (para registrar el inicio del choque)
        self._hit_logged = {}
        self.bumps = set()
        self.fallen = None
        self._sample_cache = {}

        # objetos móviles: pos (x, y, z del centro), estado
        a = arena
        p1 = a.props["pedestal1"][0]
        bp = a.props["ball_ped"][0]
        self.objects = {
            "cube": {"x": (p1[0] + p1[2]) / 2, "y": (p1[1] + p1[3]) / 2,
                     "z": ar.PEDESTAL_H + ar.CUBE_SIZE / 2, "size": ar.CUBE_SIZE,
                     "held": False, "on": "pedestal1"},
            "ball": {"x": (bp[0] + bp[2]) / 2, "y": (bp[1] + bp[3]) / 2,
                     "z": a.height((bp[0] + bp[2]) / 2, (bp[1] + bp[3]) / 2) + ar.PEDESTAL_H
                     + ar.BALL_D / 2, "size": ar.BALL_D, "held": False, "on": "ball_ped"},
        }
        pr = ar.rect_px(*ar.PRISM_PX)
        self.prism = {"x": (pr[0] + pr[2]) / 2, "y": (pr[1] + pr[3]) / 2,
                      "half": (pr[2] - pr[0]) / 2, "lost": False}
        # al reiniciar en un nivel alto se carga el objeto de ese tramo en la garra
        self.start_held = {2: "cube", 4: "ball"}.get(level)
        self.held = None
        self._servos = {}
        self.on_frame = None            # función llamada cada FRAME_S de tiempo simulado
        self._next_frame = 0.0
        self._install()

    # ------------------------------------------------------------ conexión con los fakes
    def _install(self):
        w = self.world
        w.reset()
        w.sensors["light"] = self.probe_light
        w.sensors["color"] = self.probe_color
        w.sensors["rgb"] = self.probe_rgb
        w.sensors["pitch"] = lambda: self.pitch + (self.rng.gauss(0, 0.3) * self.noise)
        w.sensors["distance"] = lambda: -1
        w.sensors["offset"] = lambda: 0
        w.sensors["line_sta"] = self._line_sta
        w.rot_z = 0.0
        self._last_yaw = self.yaw

    def place_start_object(self):
        """Pone el objeto del nivel en la garra (llamar después de cerrar la garra)."""
        if self.start_held:
            self.held = self.start_held
            self.objects[self.held]["held"] = True
            self.objects[self.held]["on"] = None
            self._carry()

    # ------------------------------------------------------------ reloj
    def sleep(self, dt):
        steps = max(1, int(round(dt / DT)))
        for _ in range(steps):
            self.step(DT)
            self.world.t += DT
            if self.on_frame is not None and self.world.t >= self._next_frame:
                self._next_frame = self.world.t + FRAME_S
                self.on_frame()

    # ------------------------------------------------------------ cinemática
    def _track_target(self, power):
        if abs(power) < self.deadband:
            return 0.0
        return self.vmax * power / 100.0

    def step(self, dt):
        if self.fallen:
            raise Fallen(self.fallen)
        b = self.body
        em = self.world.em
        left_p = em[0] * self.cfg.LEFT_SIGN            # potencia con convención "adelante +"
        right_p = em[1] * self.cfg.RIGHT_SIGN
        load = self.slope_k * math.sin(math.radians(self.pitch))
        tl = self._track_target(left_p) * self.bias
        tr = self._track_target(right_p) / self.bias
        if tl != 0:
            tl -= load
        if tr != 0:
            tr -= load
        a = dt / self.tau
        v_old = (self.vl + self.vr) / 2.0
        self.vl += (tl - self.vl) * min(1.0, a)
        self.vr += (tr - self.vr) * min(1.0, a)
        acc = ((self.vl + self.vr) / 2.0 - v_old) / dt
        self.acc += (acc - self.acc) * min(1.0, dt / 0.03)
        self._check_tip()
        # encoders: giro de la rueda (sin patinaje); convención de signo de mbot2
        d = b["sprocket_d"]
        dl = self.vl * dt / (math.pi * d) * 360.0
        dr = self.vr * dt / (math.pi * d) * 360.0
        self.world.enc[0] += dl * self.cfg.LEFT_SIGN
        self.world.enc[1] += dr * self.cfg.RIGHT_SIGN
        # movimiento real sobre el piso
        v = self.slip_true * (self.vl + self.vr) / 2.0
        tw = (b["robot_w"] - b["belt_w"]) * self.skid
        w = self.slip_true * (self.vr - self.vl) / tw          # rad/s, antihorario
        nyaw = self.yaw + math.degrees(w * dt)
        h = math.radians((self.yaw + nyaw) / 2.0)
        nx = self.x + v * dt * math.cos(h)
        ny = self.y + v * dt * math.sin(h)
        cos_p = math.cos(math.radians(self.pitch))
        nx = self.x + (nx - self.x) * cos_p
        ny = self.y + (ny - self.y) * cos_p
        if self._blocked(nx, ny, nyaw):
            self.vl *= 0.3
            self.vr *= 0.3
            self.acc = 0.0              # el golpe contra un prop no cuenta para volcar
            nx, ny, nyaw = self.x, self.y, self.yaw
        self._move_to(nx, ny, nyaw)
        self.t_phys += dt
        # giroscopio del CyberPi: rot_z = rumbo antihorario (HEADING_SIGN = -1 en config)
        dyaw = self.yaw - self._last_yaw
        self._last_yaw = self.yaw
        self.world.rot_z += dyaw + self.gyro_drift / 60.0 * dt
        if self.world.servos != self._servos:
            self._servos = dict(self.world.servos)
            self.update_gripper()
        if self.held:
            self._carry()
        self._push_prism()

    def _move_to(self, x, y, yaw):
        a = self.arena
        zc = a.height(x, y)
        if zc is None:
            self.x, self.y, self.yaw = x, y, yaw
            self._fall("salió de la arena")
            return
        if abs(zc - self.z) > 3.0:
            if zc < self.z:
                self.x, self.y, self.yaw = x, y, yaw
                self._fall("se cayó del nivel (%.0f cm)" % (self.z - zc))
                return
        self.x, self.y, self.yaw = x, y, yaw
        self.z = zc
        self._update_pitch()
        self._check_stairs()
        self._check_bumps()

    def _update_pitch(self):
        L = self.body["track_len"]
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        hf = self.arena.height(self.x + c * L / 2, self.y + s * L / 2)
        hr = self.arena.height(self.x - c * L / 2, self.y - s * L / 2)
        # una punta sin piso debajo (borde de un nivel) no cambia el pitch: queda apoyado al centro
        # (la escalera tiene ~34°: con la punta a L/2 = 9.5 cm la diferencia llega a ~6.5 cm)
        lim = L / 2.0 + 1.0
        if hf is None or abs(hf - self.z) > lim:
            hf = self.z
        if hr is None or abs(hr - self.z) > lim:
            hr = self.z
        self.pitch = math.degrees(math.atan2(hf - hr, L))

    def com(self):
        """Centro de masa (adelante, altura) en cm, en el marco del robot: chasis + brazo."""
        b = self.body
        m = b.get("arm_mass", 0.15)
        h, _ = self.arm_h()
        x = (1 - m) * b.get("com_fwd", -1.0) + m * self.claw_reach()
        z = (1 - m) * b.get("com_h", 6.5) + m * h
        return x, z

    def tip_margin(self):
        """cm entre el punto donde la gravedad + la inercia cruzan el piso y la punta de la oruga
        más cercana (negativo = se vuelca). Positivo hacia adelante = margen trasero."""
        L = self.body["track_len"]
        xc, zc = self.com()
        g = 981.0
        p = math.radians(self.pitch)
        x_hit = xc - zc * (g * math.sin(p) + self.acc) / (g * math.cos(p))
        return min(x_hit + L / 2.0, L / 2.0 - x_hit), x_hit

    def _check_tip(self):
        margin, x_hit = self.tip_margin()
        self.min_tip = min(self.min_tip, margin)
        if margin < 0:
            self._fall("se volcó hacia %s (pitch %.0f°, aceleración %.0f cm/s²)"
                       % ("atrás" if x_hit < 0 else "adelante", self.pitch, self.acc))

    def _check_stairs(self):
        """En la escalera cada oruga debe ir sobre su tramo (hay 12 cm de hueco entre ellos)."""
        a = self.arena
        if a.surface(self.x, self.y) != "stairs":
            return
        b = self.body
        off = (b["robot_w"] - b["belt_w"]) / 2.0
        s = math.sin(math.radians(self.yaw))
        c = math.cos(math.radians(self.yaw))
        for side in (-1, 1):
            # centro de la oruga (lateral) y su ancho proyectado en y (la escalera corre en x)
            cy = self.y + side * off * c
            half = b["belt_w"] / 2.0 * abs(c) + b["track_len"] / 2.0 * abs(s)
            best = 0.0
            for f in a.flights:
                ov = min(cy + half, f[3]) - max(cy - half, f[1])
                best = max(best, ov / (2 * half))
            if best < 0.4:
                self._fall("una oruga quedó fuera del tramo de la escalera")
                return

    def _check_bumps(self):
        """Recortes del pasillo y tope del nivel 1: al subirse una oruga, el rumbo se desvía."""
        a = self.arena
        fp = self.footprint()
        now = set()
        for i, r in enumerate(a.obstacles + [a.tope]):
            if ar.polys_overlap(fp, ar.rect_to_poly(r)):
                now.add(i)
                if i not in self.bumps:
                    self.yaw += self.rng.gauss(0, 2.0 if i < len(a.obstacles) else 0.8) * \
                        max(self.noise, 0.5)
        self.bumps = now

    def _fall(self, why):
        self.vl = self.vr = 0.0
        self.fallen = why
        self.log.append((self.world.t, "CAIDA: " + why))
        raise Fallen(why)

    # ------------------------------------------------------------ forma del robot
    def frame(self):
        c, s = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        return c, s

    def to_world(self, fwd, left):
        c, s = self.frame()
        return (self.x + c * fwd - s * left, self.y + s * fwd + c * left)

    def footprint(self, x=None, y=None, yaw=None):
        b = self.body
        x = self.x if x is None else x
        y = self.y if y is None else y
        yaw = self.yaw if yaw is None else yaw
        return ar.rect_poly(x, y, math.radians(yaw), b["track_len"], b["robot_w"])

    def arm_h(self):
        """Altura del centro de la garra según el ángulo del servo del brazo (cm sobre el robot)."""
        b = self.body
        ang = self.world.servos.get(self.cfg.SERVO_ARM, b["arm_zero"])
        th = (ang - b["arm_zero"]) / (b["arm_dir"] or 1)
        th = max(-90.0, min(90.0, th))
        return b["arm_pivot_h"] + b["arm_len"] * math.sin(math.radians(th)) + b["arm_grip_dz"], th

    def claw_reach(self):
        b = self.body
        h, th = self.arm_h()
        return (b["arm_pivot_fwd"] - b["ref_rot_fwd"] + b["arm_len"] * math.cos(math.radians(th))
                + b["arm_grip_dx"])

    def claw_point(self):
        """(x, y, z) del centro de la garra en la pista."""
        x, y = self.to_world(self.claw_reach(), 0.0)
        h, _ = self.arm_h()
        return x, y, self.z + h

    def claw_poly(self, x=None, y=None, yaw=None):
        b = self.body
        x = self.x if x is None else x
        y = self.y if y is None else y
        yaw = self.yaw if yaw is None else yaw
        r = self.claw_reach()
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        mid = r + (b["claw_tip"] - 2.0) / 2.0
        return ar.rect_poly(x + c * mid, y + s * mid, math.radians(yaw), b["claw_tip"] + 2.0, 6.0)

    def claw_closed(self):
        b = self.body
        ang = self.world.servos.get(self.cfg.SERVO_CLAW, b["claw_open"])
        return abs(ang - b["claw_open"]) > 15

    # ------------------------------------------------------------ choques
    def _blocked(self, x, y, yaw):
        a = self.arena
        fp = self.footprint(x, y, yaw)
        cp = self.claw_poly(x, y, yaw)
        claw_h, _ = self.arm_h()
        z = a.height(x, y)
        hit = set()
        for name, (r, h, kind) in a.props.items():
            poly = ar.rect_to_poly(r)
            base = a.height((r[0] + r[2]) / 2, (r[1] + r[3]) / 2) or 0.0
            top = base + h
            if z is not None and abs(base - z) < 6 and ar.polys_overlap(fp, poly):
                hit.add(name)
            elif ar.polys_overlap(cp, poly) and (z or 0) + claw_h - 1.5 < top:
                hit.add(name)
        for i, (p, q) in enumerate(a.walls):
            if ar.poly_seg_overlap(fp, p, q):
                hit.add("muro")
        # subir de golpe a un nivel más alto (una pared de nivel) también bloquea
        zc = a.height(x, y)
        if zc is not None and zc - self.z > 3.0:
            hit.add("borde de nivel")
        for name in hit - self.contacts:
            if self.world.t - self._hit_logged.get(name, -9) > 1.0:
                self.log.append((self.world.t, "choque: " + name))
            self._hit_logged[name] = self.world.t
        self.contacts = hit
        return bool(hit)

    def _local(self, x, y):
        """Punto de la pista -> (adelante, izquierda) respecto al centro de giro."""
        c, s = self.frame()
        dx, dy = x - self.x, y - self.y
        return dx * c + dy * s, -dx * s + dy * c

    def _cradle(self):
        """Garra abierta y baja como cuna: si el prisma cabe entre los dedos, lo lleva consigo
        (sigue las curvas). Se suelta al subir el brazo, al cerrar la garra o al retroceder."""
        pr = self.prism
        b = self.body
        h = pr["half"]
        claw_h, _ = self.arm_h()
        low = claw_h < ar.PRISM_H - 1.0 and not self.claw_closed()
        r = self.claw_reach()
        palm = r - 2.0 + h                       # el prisma apoyado contra la palma de la garra
        if pr.get("cradled"):
            back = (self.vl + self.vr) / 2.0 < -0.5
            if not low or back:
                pr["cradled"] = False
                self.log.append((self.world.t, "suelta el prisma"))
                return False
            pr["x"], pr["y"] = self.to_world(palm, 0.0)
            return True
        if not low:
            return False
        f, l = self._local(pr["x"], pr["y"])
        # dedos abiertos en V: un prisma que llega entre las puntas se encauza hacia la palma,
        # siempre que quepa entre los dedos
        fits = b.get("claw_open_w", 9.0) >= 2 * h
        if fits and abs(l) <= b.get("claw_open_w", 9.0) / 2.0 - 0.5 and \
                palm - 0.5 <= f <= r + b["claw_tip"] + h:
            pr["cradled"] = True
            self.log.append((self.world.t, "lleva el prisma en la garra"))
            pr["x"], pr["y"] = self.to_world(palm, 0.0)
            return True
        return False

    def _push_prism(self):
        pr = self.prism
        if pr["lost"]:
            return
        h = pr["half"]
        if not self._cradle():
            self._shove(h)
        zp = self.arena.height(pr["x"], pr["y"])
        if zp is None or abs(zp - self.arena.Z[3]) > 1:
            pr["lost"] = True
            pr["cradled"] = False
            self.log.append((self.world.t, "el prisma se cayó del nivel 3"))

    def _shove(self, h):
        pr = self.prism
        fp = self.footprint()
        claw_h, _ = self.arm_h()
        cp = self.claw_poly() if claw_h - 1.5 < ar.PRISM_H else None
        c, s = self.frame()
        for _ in range(12):
            poly = ar.rect_poly(pr["x"], pr["y"], 0.0, 2 * h, 2 * h)
            if not (ar.polys_overlap(fp, poly) or (cp and ar.polys_overlap(cp, poly))):
                break
            pr["x"] += c * 0.25
            pr["y"] += s * 0.25
            if not pr.get("pushed"):
                pr["pushed"] = True
                self.log.append((self.world.t, "empuja el prisma"))

    # ------------------------------------------------------------ garra y objetos
    def _carry(self):
        o = self.objects[self.held]
        o["x"], o["y"], o["z"] = self.claw_point()

    def update_gripper(self):
        """Llamar tras cada cambio de servo: toma o suelta objetos."""
        closed = self.claw_closed()
        if closed and self.held is None:
            cx, cy, cz = self.claw_point()
            r = self.claw_reach()
            for name, o in self.objects.items():
                # los dedos abiertos encierran el objeto: de lado basta con que quepa entre ellos;
                # hacia el frente la garra tiene ~2 cm de fondo
                f, l = self._local(o["x"], o["y"])
                side = max(0.5, (self.body.get("claw_open_w", 9.0) - o["size"]) / 2.0)
                if abs(f - r) <= 2.0 and abs(l) <= side and abs(o["z"] - cz) <= 2.0:
                    self.held = name
                    o["held"] = True
                    o["on"] = None
                    self.log.append((self.world.t, "toma %s" % self._name(name)))
                    break
        elif not closed and self.held is not None:
            self._release()
        if self.held:
            self._carry()

    def _name(self, n):
        return {"cube": "el cubo", "ball": "la pelota"}[n]

    def _release(self):
        name = self.held
        o = self.objects[name]
        self.held = None
        o["held"] = False
        a = self.arena
        x, y = o["x"], o["y"]
        bottom = o["z"] - o["size"] / 2
        where = None
        for pname in ("pedestal1", "pedestal2", "ball_ped"):
            r = a.props[pname][0]
            top = (a.height((r[0] + r[2]) / 2, (r[1] + r[3]) / 2) or 0) + ar.PEDESTAL_H
            if ar.inside(r, x, y, margin=0.5) and bottom >= top - 1.5:
                o["z"] = top + o["size"] / 2
                where = pname
                break
        if where is None and name == "ball":
            for i, (hx, hy) in enumerate(a.holes):
                top = (a.height(hx, hy) or 0) + a.props["bar"][1]
                if math.hypot(x - hx, y - hy) <= 1.25 and bottom >= top - 1.5:
                    o["x"], o["y"], o["z"] = hx, hy, top
                    where = "hole%d" % i
                    break
        if where is None:
            z = a.height(x, y)
            if z is None:
                where = "fuera"
                o["z"] = -10
            else:
                o["z"] = z + o["size"] / 2
                where = "piso"
        o["on"] = where
        self.log.append((self.world.t, "suelta %s en %s" % (self._name(name), where)))

    # ------------------------------------------------------------ Quad RGB
    def probe_pos(self, probe):
        b = self.body
        i = self.cfg.PROBES.index(probe)
        lat = (i - 1.5) * b["probe_pitch"]                      # negativo = izquierda
        return self.to_world(b["sensor_fwd"] - b["ref_rot_fwd"], -lat)

    def _samples(self, probe):
        key = (self.world.t, probe)
        if key in self._sample_cache:
            return self._sample_cache[key]
        if len(self._sample_cache) > 64:
            self._sample_cache.clear()
        x, y = self.probe_pos(probe)
        r = self.spot
        pts = [(x, y)] + [(x + r * math.cos(k * math.pi / 3), y + r * math.sin(k * math.pi / 3))
                          for k in range(6)]
        mats = [self.arena.material(px_, py_) for px_, py_ in pts]
        self._sample_cache[key] = mats
        return mats

    def probe_light(self, probe):
        mats = self._samples(probe)
        v = sum(ar.MATERIALS[m][1] for m in mats) / len(mats)
        v = v * self.ambient + self.rng.gauss(0, self.light_sigma) if self.noise else v
        return max(0, min(100, v))

    def probe_color(self, probe):
        mats = self._samples(probe)
        return ar.MATERIALS[mats[0]][0]

    def probe_rgb(self, probe):
        mats = self._samples(probe)
        acc = [0.0, 0.0, 0.0]
        for m in mats:
            for k in range(3):
                acc[k] += ar.MATERIALS[m][2][k]
        return tuple(int(c / len(mats)) for c in acc)

    def _line_sta(self):
        sta = 0
        for pr in self.cfg.PROBES:
            if self.probe_light(pr) < 45:
                sta |= self.cfg.LINE_BIT[pr]
        return sta


def patch_time(sim):
    """Reemplaza time.sleep (lo usa hal.wait) por el reloj del simulador. Devuelve el original."""
    orig = time.sleep
    time.sleep = sim.sleep
    return orig
