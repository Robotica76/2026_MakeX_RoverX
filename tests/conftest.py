"""Pone los módulos falsos del mBot2 y las carpetas de software/ en el path."""
import copy
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests" / "fakes"))
for d in ("core", "controls", "autonomous", "manual"):
    sys.path.insert(0, str(ROOT / "software" / d))

import world  # noqa: E402
import hal  # noqa: E402
import config as cfg  # noqa: E402

_P0 = copy.deepcopy(cfg.P)
_CAL0 = copy.deepcopy(cfg.LINE_CAL)


@pytest.fixture(autouse=True)
def sim(monkeypatch, tmp_path):
    world.reset()
    cfg.P.clear()
    cfg.P.update(copy.deepcopy(_P0))
    cfg.LINE_CAL.clear()
    cfg.LINE_CAL.update(copy.deepcopy(_CAL0))
    monkeypatch.setattr(cfg, "PARAMS_FILE", str(tmp_path / "rover_params.json"))
    monkeypatch.setattr(hal.time, "sleep", world.advance)
    hal.init()
    yield world
