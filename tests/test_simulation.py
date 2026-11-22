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
    assert min(ids) == 4


def test_level3_nominal_run_scores_level3():
    res = run_auto.run_once(level=3, seed=0, noise=0.0, record=False, budget=50)
    ok = {m["id"]: m["ok"] for m in res["report"]["missions"]}
    for mid in (7, 9):
        assert ok[mid], (mid, res["events"][-5:])
