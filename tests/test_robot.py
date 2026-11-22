"""Pruebas en PC del software del Rover X (con los módulos falsos de tests/fakes)."""
import ast
import subprocess
import sys
from pathlib import Path

import pytest

import world
import hal
import config as cfg
import geometry as geo
import colors
import motion
import linefollow as lf
import autonomous as auto
import missions as ms
import manual
import settings

ROOT = Path(__file__).resolve().parents[1]
WHITE, BLACK = 90, 10


def lights(**dark):
    """Luz por sonda a partir de oscuridad 0..1 (calibración por defecto 90 blanco / 10 negro)."""
    return lambda pr: WHITE - (WHITE - BLACK) * dark.get(pr, 0)


# ---------------------------------------------------------------- HAL
def test_drive_forward_uses_mbot2_sign_convention():
    hal.drive(50, 50)
    assert world.em == [50.0, -50.0]       # avanzar = drive_power(+, -)


def test_drive_clamps_power():
    hal.drive(150, -150)
    assert world.em == [100.0, 100.0]


def test_heading_positive_when_turning_right():
    hal.drive(40, -40)
    hal.wait(0.5)
    assert hal.heading() > 0


def test_builtin_line_mode_decodes_bits():
    cfg.P["line_mode"] = 0
    world.sensors["line_sta"] = lambda: cfg.LINE_BIT["L2"] | cfg.LINE_BIT["R2"]
    assert hal.line_bits() == {"L2": True, "L1": False, "R1": False, "R2": True}


# ---------------------------------------------------------------- línea delgada (analógico)
def test_thin_line_between_center_probes_is_still_seen():
    # línea de 10 mm entre L1 y R1: cada una medio cubierta
    world.sensors["light"] = lights(L1=0.5, R1=0.5)
    off, bits = hal.line_read()
    assert off == 0
    assert bits["L1"] and bits["R1"]


def test_offset_is_proportional_and_signed():
    world.sensors["light"] = lights(L1=0.6, L2=0.3)
    off_left, _ = hal.line_read()
    world.sensors["light"] = lights(L1=1.0)
    off_l1, _ = hal.line_read()
    assert off_left < off_l1 < 0
    world.sensors["light"] = lights(R1=1.0)
    assert hal.line_read()[0] > 0


def test_offset_zero_without_line():
    assert hal.offset_from({"L2": 0, "L1": 0, "R1": 0, "R2": 0}) == 0


# ---------------------------------------------------------------- geometría
def test_arm_horizontal_at_pivot_height():
    h = cfg.P["arm_pivot_h"] + cfg.P["arm_grip_dz"]
    assert geo.arm_servo(h) == pytest.approx(cfg.P["arm_zero"])
    assert geo.claw_reach(h) == pytest.approx(cfg.P["claw_fwd"])


def test_arm_direction_and_limits():
    hi = geo.arm_servo(cfg.P["arm_pivot_h"] + 4)
    lo = geo.arm_servo(cfg.P["arm_pivot_h"] - 6)
    assert (hi - lo) * cfg.P["arm_dir"] > 0
    assert 0 <= geo.arm_servo(100) <= 180
    cfg.P["arm_dir"] = 1
    assert geo.arm_servo(cfg.P["arm_pivot_h"] + 4) > cfg.P["arm_zero"]


def test_claw_gets_closer_when_arm_tilts():
    assert geo.claw_reach(cfg.P["arm_pivot_h"] + 6) < cfg.P["claw_fwd"]


def test_cm_per_deg_from_sprocket():
    cfg.P["sprocket_d"] = 360 / 3.141592653589793      # circunferencia 360 cm
    assert geo.cm_per_deg() == pytest.approx(1.0)


def test_geometry_driven_distances():
    cfg.P.update({"claw_fwd": 20, "us_fwd": 10, "ped_gap": 1, "arm_len": 8})
    h = cfg.P["arm_pivot_h"] + cfg.P["arm_grip_dz"]       # brazo horizontal
    assert geo.pedestal_us_target(h) == pytest.approx(20 - 3 - 10 + 1)
    assert geo.card_to_center(100, 112) == pytest.approx(100 + cfg.P["card_probe_fwd"] - 112)
    assert geo.detour_out() > cfg.P["robot_w"] / 2


# ---------------------------------------------------------------- cruces con memoria
def B(**on):
    return {p: on.get(p, False) for p in cfg.PROBES}


def test_junction_with_outer_probes_at_different_times_is_X():
    lf.reset()
    assert lf.track_junction(B(L1=True, L2=True), 10.0) is None
    assert lf.track_junction(B(L1=True), 11.0) is None
    assert lf.track_junction(B(R1=True, R2=True), 12.0) == "X"
    assert lf.last_junction == ("X", 10.0)


def test_single_side_branch_closes_after_window():
    lf.reset()
    lf.track_junction(B(L2=True), 0.0)
    assert lf.track_junction(B(L1=True), cfg.P["cross_win"] - 0.1) is None
    assert lf.track_junction(B(L1=True), cfg.P["cross_win"] + 0.1) == "L"


def test_junction_not_counted_twice():
    lf.reset()
    lf.track_junction(B(L2=True), 0.0)
    assert lf.track_junction(B(R2=True), 1.0) == "X"
    assert lf.track_junction(B(L2=True, R2=True), 1.5) is None   # todavía sobre el cruce
    lf.track_junction(B(L1=True), 2.0)
    lf.track_junction(B(L1=True), 4.0)                            # 2 cm en blanco: re-armado
    assert lf.track_junction(B(L2=True), 6.0) is None
    assert lf.track_junction(B(R2=True), 6.5) == "X"


def test_follow_counts_two_junctions_on_thin_line():
    # cruces en 20 cm y 50 cm; la sonda derecha llega 1.5 cm después que la izquierda
    def light(pr):
        x = hal.odo_abs()
        for c in (20, 50):
            if pr == "L2" and c <= x < c + 1.2:
                return BLACK
            if pr == "R2" and c + 1.5 <= x < c + 2.7:
                return BLACK
        return BLACK + 40 if pr in ("L1", "R1") else WHITE
    world.sensors["light"] = light
    assert lf.follow_until(lf.at_junction(2), timeout=20)
    assert 50 <= hal.odo_abs() <= 50 + cfg.P["cross_win"] + 1


def test_turn_at_junction_centers_rotation_point(monkeypatch):
    moves = []
    monkeypatch.setattr(motion, "straight", lambda cm, speed=None, timeout=None: moves.append(cm) or True)
    monkeypatch.setattr(motion, "turn", lambda deg, timeout=None: True)
    monkeypatch.setattr(motion, "spin_to_line", lambda d, timeout=3.0: True)
    lf.last_junction = ("X", hal.odo_abs() - 3.0)
    assert lf.turn_at_junction(90)
    assert moves[0] == pytest.approx(cfg.P["sensor_fwd"] - 3.0, abs=0.05)


def test_follow_gives_up_when_line_lost():
    world.sensors["light"] = lights()
    assert not lf.follow_until(lambda bits, j: False, timeout=5)
    assert world.t < 2


def test_line_left_slows_left_track():
    world.sensors["light"] = lights(L2=0.4, L1=0.8)
    lf.reset()
    lf.step(40)
    assert world.em[0] < -world.em[1]


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


# ---------------------------------------------------------------- movimientos
def test_turn_reaches_target():
    assert motion.turn(90)
    assert abs(hal.heading() - 90) <= cfg.P["turn_tol"] + 1


def test_straight_uses_encoders():
    assert motion.straight(20)
    assert hal.odometer_cm() >= 20


def test_climb_waits_for_ramp_then_flat():
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
    start = hal.odo_abs()

    def color(p):
        if p != cfg.CARD_PROBE:
            return "white"
        cm = hal.odo_abs() - start
        i = int(cm // 10)
        if i < len(order) and cm % 10 < 4:
            return order[i]
        return "white"
    world.sensors["color"] = color
    assert auto.scan_cards()
    assert [c for c, _ in auto.cards] == order
    assert auto.red_index(auto.cards) == 0


# ---------------------------------------------------------------- ejecutor y misiones
def test_runner_skips_lower_levels_and_stops_on_critical_failure(monkeypatch):
    ran = []
    monkeypatch.setattr(auto, "run_mission", lambda m: ran.append(m["id"]) or m["id"] != 10)
    assert auto.run(start_level=4) == 0
    assert ran == [10]


def test_runner_continues_after_non_critical_failure(monkeypatch):
    ran = []
    monkeypatch.setattr(auto, "run_mission", lambda m: ran.append(m["id"]) or m["id"] != 11)
    assert auto.run(start_level=4) == 100 + 150 + 100
    assert ran == [10, 11, 12, 13]


def test_abort_button_stops_runner(monkeypatch):
    monkeypatch.setattr(auto, "run_mission", lambda m: world.pressed.add("b") or True)
    auto.run(start_level=1)
    assert len(auto.results) == 1


def test_all_mission_steps_are_known_and_params_exist():
    known = {"line", "jturn", "straight", "turn", "spin", "approach_ped", "climb", "stairs",
             "claw", "arm", "hold", "detour", "cards", "drop_red", "stop_in", "wait"}
    for m in ms.MISSIONS:
        for s in m["steps"]:
            assert s[0] in known, (m["id"], s)
            if s[0] == "line":
                auto.make_cond(s[1])
                auto.val(s[2])
            if s[0] in ("claw", "arm", "approach_ped"):
                auto.val(s[1])


def test_params_are_read_at_run_time():
    cfg.P["h_bar"] = 7.5
    assert auto.val("h_bar") == 7.5
    assert auto.val(12) == 12


def test_points_match_rulebook():
    assert ms.total_points() == 1270


# ---------------------------------------------------------------- menú de ajustes
def test_tune_entries_point_to_existing_params():
    for group, key, step, lo, hi in cfg.TUNE:
        assert key in cfg.P, key
        assert lo <= cfg.P[key] <= hi, key


def test_bump_clamps_and_rounds():
    spec = ("Linea", "kp", 0.05, 0, 3)
    cfg.P["kp"] = 0.45
    assert settings.bump(spec, 1) == 0.5
    cfg.P["kp"] = 2.99
    assert settings.bump(spec, 1) == 3
    assert settings.bump(("Brazo", "arm_dir", 2, -1, 1), 1) == 1


def test_save_and_load_roundtrip():
    cfg.P["sensor_fwd"] = 9.5
    cfg.LINE_CAL["L1"] = [80, 12]
    assert settings.save()
    cfg.P["sensor_fwd"] = 1
    cfg.LINE_CAL["L1"] = [0, 0]
    assert settings.load()
    assert cfg.P["sensor_fwd"] == 9.5
    assert cfg.LINE_CAL["L1"] == [80, 12]


def test_load_ignores_unknown_keys():
    settings.apply({"P": {"no_existe": 1, "kp": 0.9}})
    assert "no_existe" not in cfg.P and cfg.P["kp"] == 0.9


def test_calibrate_line_by_spinning():
    # la línea queda bajo cada sonda en distintos ángulos al girar
    def light(pr):
        x = geo.probe_x(pr) + hal.heading() * 0.12        # desplazamiento lateral aparente
        return 15 if abs(x) < 0.6 else 85
    world.sensors["light"] = light
    assert settings.calibrate_line()
    for pr in cfg.PROBES:
        assert cfg.LINE_CAL[pr] == [85, 15]


def test_calibrate_line_fails_without_contrast():
    world.sensors["light"] = lambda pr: 80
    assert not settings.calibrate_line()


# ---------------------------------------------------------------- manual
def test_manual_forward_and_turn():
    world.axes = {"Ly": 100}
    manual.step({})
    assert world.em[0] > 0 and world.em[1] < 0   # adelante
    world.axes = {"Rx": 100}
    manual.step({})
    assert world.em[0] > 0 and world.em[1] > 0   # giro a la derecha


def test_manual_deadzone():
    world.axes = {"Ly": 3, "Rx": -4}
    manual.step({})
    assert world.em == [0.0, 0.0]


def test_manual_arm_preset_uses_geometry():
    world.pad = {"N2"}
    manual.step({})
    assert world.servos[cfg.SERVO_ARM] == pytest.approx(geo.arm_servo(cfg.P["h_bar"]))


# ---------------------------------------------------------------- bundle para mBlock
def test_bundle_is_single_valid_file_without_project_imports(tmp_path):
    out = tmp_path / "rover.py"
    subprocess.run([sys.executable, str(ROOT / "tools/scripts/bundle_for_mblock.py"),
                    str(ROOT / "software/robot/rover_x_2026.py"), "-o", str(out)], check=True)
    tree = ast.parse(out.read_text())
    imported = {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    assert imported <= {"time", "math", "json", "cyberpi", "mbot2", "mbuild", "gamepad"}
    assert not any(isinstance(n, ast.JoinedStr) for n in ast.walk(tree))   # sin f-strings
