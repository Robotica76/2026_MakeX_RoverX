"""Pone los módulos falsos del mBot2 y las carpetas de software/ en el path."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests" / "fakes"))
for d in ("core", "controls", "autonomous", "manual"):
    sys.path.insert(0, str(ROOT / "software" / d))

import world  # noqa: E402
import hal  # noqa: E402


@pytest.fixture(autouse=True)
def sim(monkeypatch):
    world.reset()
    monkeypatch.setattr(hal.time, "sleep", world.advance)
    hal.init()
    yield world
