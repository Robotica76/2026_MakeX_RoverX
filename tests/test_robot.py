"""Pruebas en PC del software del Rover X (con los módulos falsos de tests/fakes)."""
import ast
import subprocess
import sys
from pathlib import Path

import world
import hal
import config as cfg
import colors
import motion
import linefollow as lf
import autonomous as auto
import missions as ms
import manual

ROOT = Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------- HAL
def test_drive_forward_uses_mbot2_sign_convention():
    hal.drive(50, 50)
    assert world.em == [50.0, -50.0]       # avanzar = drive_power(+, -)


def test_drive_clamps_power():
    hal.drive(150, -150)
    assert world.em == [100.0, 100.0]


def test_line_bits_decodes_state():
    world.sensors["line_sta"] = lambda: cfg.LINE_BIT["L2"] | cfg.LINE_BIT["R2"]
    b = hal.line_bits()
    assert b == {"L2": True, "L1": False, "R1": False, "R2": True}


def test_heading_positive_when_turning_right():
    hal.drive(40, -40)
    hal.wait(0.5)
    assert hal.heading() > 0


# ---------------------------------------------------------------- colores
def test_nearest_color_is_brightness_tolerant():
    assert colors.nearest((100, 20, 20), cfg.COLOR_REF) == "red"
    assert colors.nearest((20, 80, 35), cfg.COLOR_REF) == "green"
    assert colors.nearest((240, 240, 240), cfg.COLOR_REF) == "white"
    assert colors.nearest((15, 15, 15), cfg.COLOR_REF) == "black"


def test_zone_color_selects_level():
    world.sensors["color"] = lambda p: "Yellow"
    assert auto.detect_level() == 2
    world.sensors["color"] = lambda p: "white"
    assert auto.detect_level() == 1


# ---------------------------------------------------------------- seguidor de línea
def test_line_left_slows_left_track():
    world.sensors["offset"] = lambda: -40       # línea a la izquierda
    lf.reset()
    lf.step(40)
    left, right = world.em[0], -world.em[1]
    assert left < right


def test_follow_until_intersection_counts_crossings():
    seq = iter([6] * 5 + [15] * 3 + [6] * 5 + [15] * 3 + [6] * 50)
    world.sensors["line_sta"] = lambda: next(seq)
    assert lf.follow_until(lf.at_intersection(2), timeout=5)


def test_follow_gives_up_when_line_lost():
    world.sensors["line_sta"] = lambda: 0
    assert not lf.follow_until(lambda bits: False, timeout=5)
    assert world.t < 2


def test_green_mark_turns_at_next_intersection():
    calls = []
    orig = motion.turn
    motion.turn = lambda deg, timeout=None: calls.append(deg) or True
    try:
        state = {"i": 0}

        def color(p):
            return "green" if p == "L2" and state["i"] < 3 else "white"

        def sta():
            state["i"] += 1
            return 15 if 10 <= state["i"] < 12 else 6
        world.sensors["color"] = color
        world.sensors["line_sta"] = sta
        lf.follow_until(lambda bits: state["i"] > 40, timeout=5, green=True)
    finally:
        motion.turn = orig
    assert calls and calls[0] < 0              # verde a la izquierda -> giro a la izquierda


# ---------------------------------------------------------------- movimientos
def test_turn_reaches_target():
    assert motion.turn(90)
    assert abs(hal.heading() - 90) <= cfg.TURN_TOL_DEG + 1


def test_straight_uses_encoders():
    assert motion.straight(20)
    assert hal.odometer_cm() >= 20


def test_climb_waits_for_ramp_then_flat():
    # pitch: plano 0.3 s, rampa 1 s, plano después
    world.sensors["pitch"] = lambda: 20 if 0.3 < world.t < 1.3 else 0
    assert motion.climb()
    assert world.t > 1.3


def test_climb_times_out_without_ramp():
    assert not motion.climb(timeout=1.0)


# ---------------------------------------------------------------- tarjetas
def test_red_index():
    assert auto.red_index([("blue", 0), ("red", 10), ("green", 20)]) == 1
    assert auto.red_index([("blue", 0)]) == -1


def test_scan_cards_records_six_colors_in_order():
    order = ["red", "yellow", "green", "blue", "cyan", "purple"]

    def color(p):
        if p != cfg.CARD_PROBE:
            return "white"
        cm = hal.odometer_cm()
        i = int(cm // 10)
        if i < len(order) and cm % 10 < 4:
            return order[i]
        return "white"
    world.sensors["color"] = color
    assert auto.scan_cards()
    assert [c for c, _ in auto.cards] == order
    assert auto.red_index(auto.cards) == 0


# ---------------------------------------------------------------- ejecutor
def test_runner_skips_lower_levels_and_stops_on_critical_failure(monkeypatch):
    ran = []

    def fake(m):
        ran.append(m["id"])
        return m["id"] != 10                     # falla "Evitar torre" (crítica)
    monkeypatch.setattr(auto, "run_mission", fake)
    pts = auto.run(start_level=4)
    assert ran == [10]
    assert pts == 0


def test_runner_continues_after_non_critical_failure(monkeypatch):
    ran = []

    def fake(m):
        ran.append(m["id"])
        return m["id"] != 11
    monkeypatch.setattr(auto, "run_mission", fake)
    pts = auto.run(start_level=4)
    assert ran == [10, 11, 12, 13]
    assert pts == 100 + 150 + 100


def test_abort_button_stops_runner(monkeypatch):
    monkeypatch.setattr(auto, "run_mission", lambda m: world.pressed.add("b") or True)
    auto.run(start_level=1)
    assert len(auto.results) == 1


def test_all_mission_steps_are_known():
    known = {"line", "straight", "straight_slow", "turn", "spin", "approach", "climb",
             "stairs", "claw", "lift", "hold", "detour", "cards", "drop_red", "stop_in", "wait"}
    for m in ms.MISSIONS:
        for s in m["steps"]:
            assert s[0] in known, (m["id"], s)
            if s[0] == "line":
                auto.make_cond(s[1])


def test_points_match_rulebook():
    assert ms.total_points() == 1370 - 100      # 1270 sin el bono de alianza


# ---------------------------------------------------------------- manual
def test_manual_forward_and_turn():
    world.axes = {"Ly": 100}
    manual.step({})
    assert world.em[0] > 0 and world.em[1] < 0   # adelante
    world.axes = {"Rx": 100}
    manual.step({})
    assert world.em[0] > 0 and world.em[1] > 0   # giro a la derecha (izq +, der -)


def test_manual_deadzone():
    world.axes = {"Ly": 3, "Rx": -4}
    manual.step({})
    assert world.em == [0.0, 0.0]


def test_manual_claw_and_arm_presets():
    world.pad = {"N2"}
    manual.step({})
    assert world.servos[cfg.SERVO_LIFT] == cfg.LIFT_BAR


# ---------------------------------------------------------------- bundle para mBlock
def test_bundle_is_single_valid_file_without_project_imports(tmp_path):
    out = tmp_path / "rover.py"
    subprocess.run([sys.executable, str(ROOT / "tools/scripts/bundle_for_mblock.py"),
                    str(ROOT / "software/robot/rover_x_2026.py"), "-o", str(out)], check=True)
    tree = ast.parse(out.read_text())
    imported = {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    assert imported <= {"time", "cyberpi", "mbot2", "mbuild", "gamepad"}
    src = out.read_text()
    assert "f'" not in src and 'f"' not in src  # sin f-strings (MicroPython viejo)
