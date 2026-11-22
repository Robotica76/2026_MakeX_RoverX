"""Simulador EN VIVO: corre el programa de competencia real (software/robot/rover_x_2026.py) con
los botones del CyberPi y la recolocación del equipo.

    python3 simulation/live.py            # abre http://127.0.0.1:8770
    python3 simulation/live.py --level 2 --noise 1 --port 8771 --no-open

En la página:
- **Joystick ↑ ↓ ← →**: elegir el nivel en la pantalla de inicio (↑ 1, ↓ 2, ← 3, → 4).
- **B**: arrancar el tramo (cfg.START_BUTTON). **A**: abortar durante el autónomo
  (cfg.ABORT_BUTTON) o control manual en la pantalla de inicio. **Centro**: ajustes.
- **Colocar en la zona**: lo que hace el equipo al pedir reinicio: coloca el robot en la zona del
  nivel elegido en la pantalla (si está corriendo, primero lo aborta; si se cayó, lo levanta).
  Regla del cubo: colocarlo en el nivel 2 o más con el cubo en la garra se lo quita.
- **Pausa**, **velocidad** y **Nueva corrida** (nivel, semilla, ruido).

Solo usa la biblioteca estándar de Python.
"""

import argparse
import json
import sys
import threading
import time
import types
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import run_auto as ra          # pone las rutas de software/ y tests/fakes, y P0 / CAL0
from run_auto import ROOT
from run_auto import world, cfg, geo, hal, lf, gripper, autonomous, ms
import arena as ar
import physics
import referee
import viewer

REAL_SLEEP = time.sleep


class Stop(BaseException):
    """Termina el hilo del programa (nueva corrida). No la atrapa el código del robot."""


class Session:
    def __init__(self, level=1, seed=0, noise=0.0, line_w=2.0):
        import copy
        cfg.P.clear()
        cfg.P.update(copy.deepcopy(ra.P0))
        cfg.LINE_CAL.clear()
        cfg.LINE_CAL.update(copy.deepcopy(ra.CAL0))
        cfg.SEGMENTS = True
        import tempfile
        cfg.PARAMS_FILE = str(Path(tempfile.gettempdir()) / "rover_live_params.json")
        try:                                # cada corrida empieza con los valores de config.py
            Path(cfg.PARAMS_FILE).unlink()
        except OSError:
            pass
        self.level = level
        self.params = {"level": level, "seed": seed, "noise": noise, "line_w": line_w}
        self.arena = ar.Arena(line_w=line_w, seed=seed)
        self.log = []
        self.sim = physics.Sim(world, cfg, geo, self.arena, level=level, noise=noise, seed=seed,
                               log=self.log)
        self.sim.restarts = []
        self.sim.falls = 0
        self.ref = referee.Referee(self.sim, ms.MISSIONS, level)
        self.frames = []
        self.label = ""
        self.lock = threading.Lock()
        self.speed = 1.0
        self.paused = False
        self.stop = False
        self.releases = []                  # (t, botón) para soltar botones
        self.pending = []                   # órdenes del usuario para el hilo del programa
        self.state = ""
        self.aborting = False
        self.placed_at = None
        self.results = []
        self.sim.on_frame = self._on_frame
        self.thread = threading.Thread(target=self._run, daemon=True)

    # ------------------------------------------------------------ reloj y cuadros
    def sleep(self, dt):
        if self.stop:
            raise Stop()
        while self.paused and not self.stop:
            REAL_SLEEP(0.05)
        self._service()
        lvl = self.screen_level()
        if self.aborting and lvl is not None:
            self.aborting = False
        # colocado por el equipo y luego se eligió otro nivel con el joystick: mover a esa zona
        if lvl is None:
            self.placed_at = None
        elif self.placed_at is not None and lvl != self.placed_at:
            self.place(lvl)
        self.sim.sleep(dt)
        for t, b in list(self.releases):
            if world.t >= t:
                world.pressed.discard(b)
                self.releases.remove((t, b))
        REAL_SLEEP(dt / max(0.1, self.speed))

    def _on_frame(self):
        self.ref.update()
        with self.lock:
            self.frames.append(ra.frame_of(self.sim, self.label))

    def press(self, b, hold=0.15):
        world.pressed.add(b)
        self.releases.append((world.t + hold, b))

    # ------------------------------------------------------------ equipo
    def zone_level(self):
        climbed = sum(1 for mid in (3, 6, 9) if mid in self.ref.done)
        return max(1, min(4, 1 + climbed))

    def screen_level(self):
        """Nivel elegido en la pantalla de inicio, o None si no está en ella."""
        lab = world.labels.get(0, "")
        if lab.startswith("Nivel "):
            try:
                return int(lab.split()[1])
            except ValueError:
                return None
        return None

    def place(self, lvl):
        world.pressed.clear()
        self.placed_at = lvl
        self.sim.place(lvl)
        self.sim.restarts.append(lvl)
        self.log.append((world.t, "EQUIPO: coloca el robot en la zona del nivel %d" % lvl))
        hal.clear_abort()

    def command(self, b):
        with self.lock:
            self.pending.append(b)

    def _service(self):
        """Órdenes del usuario que se aplican en el hilo del programa (entre pasos de física)."""
        with self.lock:
            want = "place" in self.pending
        if not want:
            return
        lvl = self.screen_level()
        if lvl is not None:
            with self.lock:
                self.pending.remove("place")
            self.place(lvl)
        elif not self.aborting:
            self.aborting = True
            self.press(cfg.ABORT_BUTTON, hold=0.3)     # está corriendo: abortar primero

    # ------------------------------------------------------------ hilo del programa
    def _program(self):
        src = (ROOT / "software" / "robot" / "rover_x_2026.py").read_text(encoding="utf-8")
        src = src.rstrip()
        if src.endswith("main()"):
            src = src[:-len("main()")]
        ns = {"__name__": "rover_x_2026"}
        exec(compile(src, "rover_x_2026.py", "exec"), ns)
        return ns["main"]

    def _run(self):
        hal.time = types.SimpleNamespace(sleep=self.sleep)
        orig_rm = autonomous.run_mission

        def run_mission(m):
            self.label = "M%d %s" % (m["id"], m["name"])
            self.log.append((world.t, "PROGRAMA: empieza " + self.label))
            ok = orig_rm(m)
            self.log.append((world.t, "PROGRAMA: %s %s" % (self.label, "OK" if ok else "FALLA")))
            self.results.append((m["id"], ok))
            return ok
        autonomous.run_mission = run_mission
        orig_rs = autonomous.run_step

        def run_step(st):
            self.sim.program_step = st[0]
            return orig_rs(st)
        autonomous.run_step = run_step
        autonomous.completed.clear()
        main = self._program()
        try:
            if self.sim.start_held:
                hal.servo(cfg.SERVO_CLAW, cfg.P["claw_ball"])
                self.sim.place_start_object()
            while not self.stop:
                try:
                    main()                       # el programa de competencia (no termina solo)
                except physics.Fallen as e:
                    self.sim.falls += 1
                    self.log.append((world.t, "CAIDA: %s" % e))
                    self.state = "Se cayó: %s. Usa Colocar en la zona" % e
                    while not self.stop:         # esperar a que el equipo lo levante
                        with self.lock:
                            if "place" in self.pending:
                                self.pending.remove("place")
                                break
                        REAL_SLEEP(0.05)
                    self.place(self.screen_level() or self.zone_level())
                    self.state = ""
        except Stop:
            pass
        finally:
            autonomous.run_mission = orig_rm
            autonomous.run_step = orig_rs

    # ------------------------------------------------------------ estado para la página
    def snapshot(self, nf, ne):
        with self.lock:
            frames = self.frames[nf:]
            events = [(round(t, 2), e) for t, e in self.log[ne:]]
        prog = {}
        for mid, ok in list(self.results):
            prog[mid] = prog.get(mid, False) or ok
        return {
            "frames": frames, "events": events,
            "status": {"state": self.state,
                       "t": round(world.t, 1), "speed": self.speed, "paused": self.paused,
                       "restarts": len(self.sim.restarts), "falls": self.sim.falls,
                       "screen": [world.labels.get(k, "") for k in range(4)]},
            "judge": self.ref.snapshot(), "program": prog,
        }

    def init_data(self):
        return {
            "live": True, "report": {"missions": [], "program": {}, "program_points": 0, "total": 0,
                                     "notes": [], "warnings": ra.geometry_warnings(), "error": None},
            "frames": [], "events": [], "arena": self.arena.to_json(),
            "robot": {k: cfg.P[k] for k in ("track_len", "robot_w", "belt_w", "sensor_fwd",
                                            "ref_rot_fwd", "probe_pitch", "claw_tip",
                                            "arm_pivot_fwd", "arm_pivot_h")},
            "params": dict(self.params, prog={}, true={}, cards=self.arena.card_colors,
                           ramp_deg=None),
            "missions": [{"id": m["id"], "name": m["name"], "pts": m["pts"], "level": m["level"]}
                         for m in ms.MISSIONS],
        }


SESSION = None


def new_session(**kw):
    global SESSION
    if SESSION is not None:
        SESSION.stop = True
        SESSION.paused = False
        SESSION.thread.join(timeout=5)
    SESSION = Session(**kw)
    SESSION.thread.start()
    return SESSION


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype="application/json"):
        data = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(200)
        self.send_header("Content-Type", ctype + "; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        s = SESSION
        if u.path == "/":
            html = viewer.TEMPLATE.read_text(encoding="utf-8")
            html = html.replace("/*__DATA__*/null", json.dumps(s.init_data(), ensure_ascii=False))
            return self._send(html, "text/html")
        if u.path == "/state":
            return self._send(json.dumps(s.snapshot(int(q.get("f", 0)), int(q.get("e", 0))),
                                         ensure_ascii=False))
        self.send_error(404)

    def do_POST(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        s = SESSION
        cmd = u.path.strip("/")
        if cmd == "btn" and q.get("k") in ("up", "down", "left", "right", "middle", "a", "b"):
            s.press(q["k"], hold=0.25)
        elif cmd == "place":
            s.command("place")
        elif cmd == "pause":
            s.paused = not s.paused
        elif cmd == "speed":
            s.speed = float(q.get("v", 1))
        elif cmd == "new":
            new_session(level=int(q.get("level", 1)), seed=int(q.get("seed", 0)),
                        noise=float(q.get("noise", 0)))
            return self._send(json.dumps({"reload": True}))
        else:
            return self.send_error(404)
        self._send(json.dumps({"ok": True}))


def main(argv=None):
    ap = argparse.ArgumentParser(description="Simulador en vivo de Rover X")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--level", type=int, default=1)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--noise", type=float, default=0.0)
    ap.add_argument("--no-open", action="store_true")
    a = ap.parse_args(argv)
    new_session(level=a.level, seed=a.seed, noise=a.noise)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    url = "http://127.0.0.1:%d/" % a.port
    print("Simulador en vivo: %s  (Ctrl+C para salir)" % url)
    if not a.no_open:
        webbrowser.open(url)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    sys.exit(main())
