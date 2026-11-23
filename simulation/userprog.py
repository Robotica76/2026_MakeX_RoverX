"""Correr en el simulador un programa SUBIDO por el usuario (.py o .mblock en modo Python).

Programas como los del equipo (FuncionesEnPython.mblock, calibrations.py) usan el módulo
`event` de mBlock: @event.start y @event.is_press('a') corren EN PARALELO en el CyberPi. Aquí
cada manejador es un hilo y todos comparten un reloj simulado: la física avanza solo cuando
TODOS los hilos del programa están esperando (time.sleep), hasta el que despierta primero.
Así el resultado no depende de la velocidad de la PC.

Módulos disponibles para el programa: cyberpi, mbot2, mbuild, gamepad, event, time, math,
random y los del proyecto (hal, config, motion, ...).
"""

import io
import json
import sys
import threading
import time
import types
import zipfile

REAL_SLEEP = time.sleep
REAL_TIME = time.time


class Stop(BaseException):
    """Termina los hilos del programa (nueva corrida). El código del robot no la atrapa."""


def source_from_upload(name, data):
    """Bytes de un .py o .mblock -> (código Python, descripción). Lanza ValueError si no hay
    código Python (un .mblock solo de bloques)."""
    if name.lower().endswith(".mblock") or data[:2] == b"PK":
        try:
            z = zipfile.ZipFile(io.BytesIO(data))
            proj = json.loads(z.read("project.json").decode("utf-8"))
        except (zipfile.BadZipFile, KeyError, ValueError) as e:
            raise ValueError("no es un .mblock válido (%s)" % e)
        best = None
        for t in proj.get("targets", []):
            code = (t.get("codes") or {}).get("python") or ""
            if code.strip() and (best is None or len(code) > len(best[1])):
                best = (t.get("name", "?"), code)
        if best is None:
            nb = sum(len(t.get("blocks") or {}) for t in proj.get("targets", []))
            raise ValueError("el .mblock no tiene código Python (%d bloques). Por ahora el "
                             "simulador corre programas en modo Python." % nb)
        return best[1], "%s (objeto '%s' del .mblock)" % (name, best[0])
    try:
        return data.decode("utf-8"), name
    except UnicodeDecodeError:
        return data.decode("latin-1"), name


class Runner:
    """Hilos del programa + reloj simulado. `step(dt)` avanza la física (lo da la sesión)."""

    def __init__(self, world, step, log, on_idle=None, speed=lambda: 1.0, paused=lambda: False):
        self.world = world
        self.step = step                # avanza la física dt segundos
        self.log = log
        self.on_idle = on_idle          # se llama en cada vuelta del reloj (botones, reinicios)
        self.speed = speed
        self.paused = paused
        self.cv = threading.Condition()
        self.wake = {}                  # id de hilo -> hora de despertar (None = corriendo)
        self.names = {}
        self.stop = False
        self.start_handlers = []
        self.press_handlers = {}        # botón -> función
        self.running_press = set()
        self._prev_pressed = set()
        self.error = None

    # ------------------------------------------------------------ reloj
    def mine(self):
        return threading.get_ident() in self.wake

    def sleep(self, dt):
        if not self.mine():
            return REAL_SLEEP(dt)
        me = threading.get_ident()
        with self.cv:
            self.wake[me] = self.world.t + max(0.0, dt)
            self.cv.notify_all()
            while self.wake.get(me) is not None:
                if self.stop:
                    raise Stop()
                self.cv.wait(0.05)
        if self.stop:
            raise Stop()

    def now(self):
        return self.world.t if self.mine() else REAL_TIME()

    def _spawn(self, fn, name):
        started = threading.Event()

        def body():
            me = threading.get_ident()
            with self.cv:
                self.wake[me] = None
                self.names[me] = name
            started.set()
            try:
                fn()
            except Stop:
                pass
            except Exception as e:          # noqa: BLE001 - se informa al usuario
                self._report(name, e)
            finally:
                with self.cv:
                    self.wake.pop(me, None)
                    self.names.pop(me, None)
                    self.running_press.discard(name)
                    self.cv.notify_all()
        th = threading.Thread(target=body, daemon=True)
        th.start()
        started.wait(2)

    def _report(self, name, e):
        import traceback
        tb = traceback.extract_tb(e.__traceback__)
        where = ""
        for fr in reversed(tb):
            if fr.filename == "programa":
                where = " (línea %d)" % fr.lineno
                break
        self.log.append((self.world.t, "ERROR en %s%s: %s: %s" % (name, where, type(e).__name__, e)))
        if self.error is None:
            self.error = "%s%s: %s" % (type(e).__name__, where, e)

    def clock_loop(self):
        """Hilo del reloj: espera a que todos los hilos duerman y avanza la física."""
        while not self.stop:
            while self.paused() and not self.stop:
                REAL_SLEEP(0.05)
            with self.cv:
                deadline = REAL_TIME() + 2.0
                while any(w is None for w in self.wake.values()) and not self.stop:
                    self.cv.wait(0.02)
                    if REAL_TIME() > deadline:       # un hilo que no duerme (bucle sin sleep)
                        break
                pending = [w for w in self.wake.values() if w is not None]
            target = min(pending) if pending else self.world.t + 0.02
            dt = max(0.0, min(target - self.world.t, 0.1))
            if dt > 0:
                self.step(dt)
                REAL_SLEEP(dt / max(0.1, self.speed()))
            else:
                REAL_SLEEP(0)
            if self.on_idle:
                self.on_idle()
            self._buttons()
            with self.cv:
                for k, w in list(self.wake.items()):
                    if w is not None and w <= self.world.t + 1e-9:
                        self.wake[k] = None
                self.cv.notify_all()

    def _buttons(self):
        now = set(self.world.pressed)
        for k in now - self._prev_pressed:
            fn = self.press_handlers.get(k)
            if fn is not None and k not in self.running_press:
                self.running_press.add(k)
                self._spawn(fn, k)
        self._prev_pressed = now

    # ------------------------------------------------------------ programa
    def event_module(self):
        ev = types.ModuleType("event")
        runner = self

        def start(fn):
            runner.start_handlers.append(fn)
            return fn

        def is_press(key):
            def deco(fn):
                runner.press_handlers[key] = fn
                return fn
            return deco

        def noop(*a, **k):
            if a and callable(a[0]):
                return a[0]
            return lambda fn: fn
        ev.start = start
        ev.is_press = is_press
        for n in ("is_release", "greater_than", "smaller_than", "receive", "is_shake",
                  "is_tiltleft", "is_tiltright", "is_tiltforward", "is_tiltback"):
            setattr(ev, n, noop)
        return ev

    def run(self, source):
        """Ejecuta el programa: el cuerpo del archivo en un hilo; al terminar, los
        manejadores @event.start (cada uno en su hilo)."""
        sys.modules["event"] = self.event_module()
        code = compile(source, "programa", "exec")
        ns = {"__name__": "__main__"}

        def body():
            exec(code, ns)
            for i, fn in enumerate(list(self.start_handlers)):
                self._spawn(fn, "inicio %d" % (i + 1))
        self._spawn(body, "programa")
        self.clock_loop()
