"""Simulador en vivo DENTRO DEL NAVEGADOR (Pyodide, en un Web Worker).

Es simulation/live.py sin servidor ni hilos: la misma Session (programa de competencia o control
manual) corre en el hilo del worker. El reloj real y las órdenes de la página pasan por
js.simSleep(ms), que espera con Atomics.wait sobre memoria compartida y devuelve las órdenes
pendientes (botones, control, colocar, pausa, velocidad). El estado se manda a la página con
postMessage cada ~0.1 s.
"""

import json
import time

import js            # ámbito global del worker
import live
from run_auto import world

S = None
_sent = {"f": 0, "e": 0, "t": 0.0}


def _now():
    return js.performance.now() / 1000.0


def _post(force=False):
    if S is None:
        return
    now = _now()
    if not force and now - _sent["t"] < 0.1:
        return
    _sent["t"] = now
    snap = S.snapshot(_sent["f"], _sent["e"])
    _sent["f"] += len(snap["frames"])
    _sent["e"] += len(snap["events"])
    js.postMessage(json.dumps({"type": "snap", "s": snap}, ensure_ascii=False))


def _handle(cmds):
    for c in cmds:
        k = c.get("c")
        if k == "btn" and c.get("k") in ("up", "down", "left", "right", "middle", "a", "b"):
            S.press(c["k"], hold=0.25)
        elif k == "place":
            S.command("place")
        elif k == "pause":
            S.paused = not S.paused
            _post(True)
        elif k == "speed":
            S.speed = float(c.get("v", 1))
        elif k == "pad":
            if c.get("on", True):
                world.pad = set(c.get("keys", []))
                world.axes = {a: float(c.get(a, 0)) for a in ("Lx", "Ly", "Rx", "Ry")}
            else:
                world.pad = set()
                world.axes = {}


_clock = {"due": None}


def web_sleep(dt):
    """Reemplaza time.sleep: espera dt segundos reales y atiende la página. Lleva un reloj:
    solo espera lo que falta (el cálculo de la física también toma tiempo en el navegador)."""
    now = _now()
    due = _clock["due"]
    if due is None or now - due > 0.25:      # muy atrasado (o primer paso): no acumular deuda
        due = now
    due += max(0.0, dt)
    _clock["due"] = due
    cmds = js.simSleep(max(0.0, due - now) * 1000.0)
    if cmds and cmds != "[]":
        _handle(json.loads(cmds))
    _post()


def main(params_json):
    global S
    p = json.loads(params_json or "{}")
    live.REAL_SLEEP = web_sleep
    time.sleep = web_sleep
    S = live.Session(level=int(p.get("level", 1)), seed=int(p.get("seed", 0)),
                     noise=float(p.get("noise", 0)),
                     mode="manual" if p.get("mode") == "manual" else "comp")
    live.SESSION = S
    js.postMessage(json.dumps({"type": "init", "data": S.init_data()}, ensure_ascii=False))
    S._run()                 # no termina (el programa de competencia espera en su menú)
