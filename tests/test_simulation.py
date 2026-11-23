"""Pruebas del simulador (simulation/): la pista, la física y el recorrido nominal."""
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

import arena as ar  # noqa: E402
import run_auto  # noqa: E402


@pytest.fixture(autouse=True)
def real_sleep(monkeypatch):
    # conftest cambia time.sleep por el mundo mínimo; el simulador pone el suyo en cada corrida
    monkeypatch.setattr(time, "sleep", time.sleep)


def test_arena_scale_and_materials():
    a = ar.Arena()
    assert a.nx * a.RES == pytest.approx(244, abs=0.5)
    x, y = ar.px(73.5, 150)                     # vertical que sale de la X roja
    assert ar.MATERIALS[a.material(x, y)][0] == "black"
    x, y = ar.px(250, 100)                      # piso del nivel 1
    assert ar.MATERIALS[a.material(x, y)][0] == "white"
    red = [c for (_, c) in a.cards if c == "red"]
    assert len(a.cards) == 6 and len(red) == 1


def test_arena_heights_follow_ramps_and_stairs():
    a = ar.Arena()
    assert a.height(*ar.px(200, 100)) == ar.Z[1]
    assert ar.Z[1] < a.height(*ar.px(476, 100)) < ar.Z[2]       # rampa gris
    assert a.height(*ar.px(700, 100)) == ar.Z[2]
    assert ar.Z[3] < a.height(*ar.px(605, 414)) < ar.Z[4]       # escalera
    assert a.height(-1, 10) is None


def test_level1_nominal_run_scores_first_missions():
    res = run_auto.run_once(level=1, seed=0, noise=0.0, record=False, budget=75)
    ok = {m["id"]: m["ok"] for m in res["report"]["missions"]}
    for mid in (1, 2, 3, 4, 5):
        assert ok[mid], (mid, res["events"][-5:])
    assert not res["report"]["error"]


def test_runs_do_not_leak_state():
    a = run_auto.run_once(level=2, seed=1, noise=1.0, record=False, budget=24)
    b = run_auto.run_once(level=2, seed=1, noise=1.0, record=False, budget=24)
    assert a["report"]["total"] == b["report"]["total"]


def test_referee_ignores_missions_before_start_level():
    res = run_auto.run_once(level=2, seed=0, noise=0.0, record=False, budget=5)
    ids = [m["id"] for m in res["report"]["missions"]]
    assert min(ids) == 5          # la misión 4 es del tramo 1 (antes del reinicio en amarillo)


def test_level3_nominal_run_scores_level3():
    res = run_auto.run_once(level=3, seed=0, noise=0.0, record=False, budget=50)
    ok = {m["id"]: m["ok"] for m in res["report"]["missions"]}
    for mid in (7, 9):
        assert ok[mid], (mid, res["events"][-5:])


def test_calibrations_program_runs_sequence(monkeypatch):
    """calibrations.py (programa de prueba del equipo) hace su secuencia en el simulador:
    20 cm, giros de 90°, 10 + 20 cm y un círculo de 40 cm, en piso plano."""
    import math
    import types
    import copy
    from run_auto import world, cfg, geo
    import physics
    ev = types.ModuleType("event")
    ev.start = lambda f: f
    ev.is_press = lambda k: (lambda f: f)
    monkeypatch.setitem(sys.modules, "event", ev)
    cfg.P.clear()
    cfg.P.update(copy.deepcopy(run_auto.P0))
    world.reset()
    a = ar.Arena()
    a.height = lambda x, y: 0.0
    a.surface = lambda x, y: 1
    sim = physics.Sim(world, cfg, geo, a, level=1, noise=0.0, seed=0, log=[])
    sim.x, sim.y, sim.yaw, sim.z = 0.0, 0.0, 90.0, 0.0
    sim._last_yaw = sim.yaw
    sim._blocked = lambda x, y, yaw: False
    sim._check_bumps = lambda: None
    sim._check_stairs = lambda: None
    pts = []
    sim.on_frame = lambda: pts.append((sim.x, sim.y))
    monkeypatch.setattr(time, "sleep", sim.sleep)
    monkeypatch.setattr(time, "time", lambda: world.t)
    ns = {"__name__": "calibrations"}
    src = (ROOT / "software" / "robot" / "calibrations.py").read_text(encoding="utf-8")
    exec(compile(src, "calibrations.py", "exec"), ns)
    assert ns["secuencia"]()
    # 20 al norte y 30 al oeste antes del círculo; el círculo vuelve al mismo punto
    assert pts[-1][0] == pytest.approx(-30, abs=3) and pts[-1][1] == pytest.approx(20, abs=3)
    # círculo hacia la izquierda (al sur, mirando al oeste): centro en (-30, 0), radio 20
    assert min(p[0] for p in pts) == pytest.approx(-50, abs=4)
    assert min(p[1] for p in pts) == pytest.approx(-20, abs=4)
