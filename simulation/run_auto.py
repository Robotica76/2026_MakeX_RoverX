"""Corre el área automática del robot en la pista simulada.

Usa el programa REAL (software/autonomous, controls, core) sobre los módulos falsos del mBot2.

    python3 simulation/run_auto.py                       # nivel 1, sin ruido, abre el reporte
    python3 simulation/run_auto.py --level 3 --noise 1 --seed 7
    python3 simulation/run_auto.py --runs 50 --noise 1   # Monte Carlo: % de éxito por misión
    python3 simulation/run_auto.py --set v_line=55 --set kp=0.6      # cambiar parámetros (cfg.P)
    python3 simulation/run_auto.py --true ref_rot_fwd=6  # el robot real difiere de lo configurado
    python3 simulation/run_auto.py --line-w 1.0          # línea de 10 mm (mínimo del reglamento)

Escribe build/sim/rover_sim.html (reproducción interactiva) y imprime el reporte del juez.
"""

import argparse
import copy
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "tests" / "fakes"))
for _d in ("core", "controls", "autonomous", "manual"):
    sys.path.insert(0, str(ROOT / "software" / _d))

import world  # noqa: E402
import config as cfg  # noqa: E402
import geometry as geo  # noqa: E402
import hal  # noqa: E402
import linefollow as lf  # noqa: E402
import gripper  # noqa: E402
import autonomous  # noqa: E402
import missions as ms  # noqa: E402

import arena as ar  # noqa: E402
import physics  # noqa: E402
import referee  # noqa: E402
import viewer  # noqa: E402

P0 = copy.deepcopy(cfg.P)
CAL0 = copy.deepcopy(cfg.LINE_CAL)


def parse_kv(items):
    out = {}
    for it in items or []:
        k, v = it.split("=", 1)
        try:
            out[k] = int(v)
        except ValueError:
            out[k] = float(v)
    return out


def geometry_warnings():
    """Avisos de geometría que el programa no detecta solo."""
    out = []
    P = cfg.P
    top = P["arm_pivot_h"] + P["arm_len"] + P["arm_grip_dz"]
    low = P["arm_pivot_h"] - P["arm_len"] + P["arm_grip_dz"]
    for k in ("h_floor", "h_pedestal", "h_carry", "h_bar"):
        if P[k] > top + 0.01 or P[k] < low - 0.01:
            out.append("el brazo no alcanza %s = %.1f cm (rango %.1f .. %.1f)" % (k, P[k], low, top))
    lift = P["h_carry"] - ar.CUBE_SIZE / 2 - ar.PEDESTAL_H
    if lift < referee.LIFT_CM:
        out.append("h_carry deja el cubo %.1f cm sobre el pedestal (la misión 2 pide %.0f)"
                   % (lift, referee.LIFT_CM))
    return out


def frame_of(sim, label):
    """Un cuadro de la reproducción (el visor lee estos índices)."""
    o = sim.objects
    h, _ = sim.arm_h()
    return [
        round(world.t, 2), round(sim.x, 2), round(sim.y, 2), round(sim.yaw, 1),
        round(sim.z, 1), round(sim.pitch, 1), round(h, 1), 1 if sim.claw_closed() else 0,
        list(world.leds),
        [round(o["cube"]["x"], 1), round(o["cube"]["y"], 1), round(o["cube"]["z"], 1)],
        [round(o["ball"]["x"], 1), round(o["ball"]["y"], 1), round(o["ball"]["z"], 1)],
        [round(sim.prism["x"], 1), round(sim.prism["y"], 1), 1 if sim.prism["lost"] else 0],
        label,
        [round(v, 1) for v in sim.claw_point()[:2]],
    ]


RESTART_S = 15.0            # s que tarda el equipo en pedir el reinicio y recolocar el robot
MAX_RETRIES = 2             # reintentos del mismo tramo tras una falla o una caída


def run_once(level=1, seed=0, noise=0.0, line_w=2.0, prog=None, true=None, budget=None,
             record=True, ramp_deg=None, restarts=True):
    """restarts=True: programa por tramos (cfg.SEGMENTS) y el simulador hace de equipo: al
    terminar un tramo recoloca el robot en la zona del siguiente nivel (RESTART_S), lo elige y
    arranca; si el tramo falla o el robot se cae, reintenta ese nivel (MAX_RETRIES)."""
    """Una corrida completa. Devuelve un dict con el reporte, el trazo y los eventos."""
    cfg.P.clear()
    cfg.P.update(copy.deepcopy(P0))
    cfg.P.update(prog or {})
    cfg.LINE_CAL.clear()
    cfg.LINE_CAL.update(copy.deepcopy(CAL0))
    arena = ar.Arena(line_w=line_w, seed=seed, ramp_deg=ramp_deg)
    log = []
    sim = physics.Sim(world, cfg, geo, arena, level=level, noise=noise, seed=seed,
                      true_overrides=true, log=log)
    ref = referee.Referee(sim, ms.MISSIONS, level)
    frames = []
    label = {"m": ""}

    sim.restarts = []
    sim.falls = 0

    def on_frame():
        ref.update()            # primero el juez (antes de que el equipo mueva el robot)
        if record:
            frames.append(frame_of(sim, label["m"]))

    sim.on_frame = on_frame
    orig_run_mission = autonomous.run_mission

    def run_mission(m):
        label["m"] = "M%d %s" % (m["id"], m["name"])
        log.append((world.t, "PROGRAMA: empieza " + label["m"]))
        ok = orig_run_mission(m)
        log.append((world.t, "PROGRAMA: %s %s" % (label["m"], "OK" if ok else "FALLA")))
        return ok

    autonomous.run_mission = run_mission
    orig_run_step = autonomous.run_step

    def run_step(st):
        sim.program_step = st[0]
        return orig_run_step(st)
    autonomous.run_step = run_step
    orig_sleep = physics.patch_time(sim)
    cfg.SEGMENTS = restarts
    autonomous.completed.clear()       # encendido nuevo
    all_results = []
    points = 0
    error = None
    try:
        hal.init()
        lf.reset()
        gripper.home()
        if sim.start_held:
            gripper.arm_to(cfg.P["h_carry"])
            gripper.claw(cfg.P["claw_ball"])
            sim.place_start_object()
        hal.reset_heading()
        t0 = world.t
        bud = cfg.AUTO_BUDGET_S if budget is None else budget
        lvl = level
        retries = 0
        while True:
            fell = False
            try:
                try:
                    points += autonomous.run(lvl, budget_s=bud, t_start=t0,
                                             skip=autonomous.completed)
                finally:
                    all_results.extend(autonomous.results)
            except physics.Fallen:
                sim.falls += 1
                fell = True
            # el equipo: tramo terminado -> siguiente nivel; falla o caída -> reintentar
            # (en la zona del último nivel subido, según el juez)
            if not restarts:
                break
            nxt = autonomous.next_level
            if fell or nxt == lvl:
                climbed = sum(1 for mid in (3, 6, 9) if mid in ref.done)
                nxt = max(lvl, 1 + climbed)
                retries += 1
                if retries > MAX_RETRIES:
                    break
            else:
                retries = 0
            last = ms.MISSIONS[-1]["id"]
            if nxt > 4 or (not fell and last in [mid for mid, _ in autonomous.results]):
                break
            if world.t + RESTART_S - t0 >= bud:
                break
            hal.stop()
            world.t += RESTART_S
            world.pressed.clear()
            sim.place(nxt)
            sim.restarts.append(nxt)
            log.append((world.t, "EQUIPO: reinicio, coloca en la zona del nivel %d, elige %d y arranca"
                        % (nxt, nxt)))
            hal.clear_abort()
            hal.reset_heading()
            lvl = nxt
    except Exception as e:     # un error del programa también es un resultado
        import traceback
        error = traceback.format_exc()
        log.append((world.t, "ERROR del programa: %r" % e))
    finally:
        if not sim.fallen:
            try:                    # que termine de frenar antes del veredicto final
                hal.stop()
                sim.sleep(0.5)
            except physics.Fallen:
                pass
        time.sleep = orig_sleep
        autonomous.run_mission = orig_run_mission
        autonomous.run_step = orig_run_step
    on_frame()
    rep = ref.final()
    rep["program"] = {}
    for mid, ok in all_results:
        rep["program"][mid] = rep["program"].get(mid, False) or ok
    rep["program_points"] = points
    rep["time"] = round(world.t, 1)
    rep["error"] = error
    rep["warnings"] = geometry_warnings()
    return {
        "report": rep, "frames": frames, "events": [(round(t, 2), e) for t, e in log],
        "arena": arena.to_json() if record else None,
        "robot": {k: dict(cfg.P, **physics.track_dims(cfg.P))[k] for k in ("track_len", "track_off", "nose_h", "robot_w", "belt_w", "sensor_fwd",
                                        "ref_rot_fwd", "probe_pitch", "claw_tip",
                                        "arm_pivot_fwd", "arm_pivot_h")},
        "params": {"level": level, "seed": seed, "noise": noise, "line_w": line_w,
                   "ramp_deg": ramp_deg,
                   "prog": prog or {}, "true": true or {}, "cards": arena.card_colors},
        "missions": [{"id": m["id"], "name": m["name"], "pts": m["pts"], "level": m["level"]}
                     for m in ms.MISSIONS],
    }


def print_report(res):
    rep = res["report"]
    prm = res["params"]
    print("Nivel %d  semilla %d  ruido %.1f  línea %.1f cm  tarjetas %s"
          % (prm["level"], prm["seed"], prm["noise"], prm["line_w"], ",".join(prm["cards"])))
    print("%-4s %-22s %6s %10s %8s" % ("M", "Misión", "Pts", "Programa", "Juez"))
    names = {m["id"]: m["name"] for m in res["missions"]}
    for m in rep["missions"]:
        pg = rep["program"].get(m["id"])
        pgs = "-" if pg is None else ("OK" if pg else "falla")
        print("%-4d %-22s %6d %10s %8s" % (m["id"], names[m["id"]], m["pts"], pgs,
                                            "OK" if m["ok"] else "no"))
    print("Total juez: %d   (el programa estima %d)   tiempo %.1f s   margen mínimo contra volcadura %.1f cm"
          % (rep["total"], rep["program_points"], rep["time"], rep["min_tip_cm"]))
    for n in rep["notes"] + rep["warnings"]:
        print("  ! " + n)
    if rep["error"]:
        print(rep["error"])


def monte_carlo(n, level, noise, line_w, prog, true, seed0=0, ramp_deg=None, budget=None):
    ok = {}
    totals = []
    falls = 0
    for i in range(n):
        res = run_once(level, seed0 + i, noise, line_w, prog, true, budget, record=False,
                       ramp_deg=ramp_deg)
        rep = res["report"]
        totals.append(rep["total"])
        falls += 1 if any(x.startswith("CAÍDA") for x in rep["notes"]) else 0
        for m in rep["missions"]:
            ok.setdefault(m["id"], 0)
            ok[m["id"]] += 1 if m["ok"] else 0
    names = {m["id"]: m["name"] for m in ms.MISSIONS}
    print("Monte Carlo: %d corridas, nivel %d, ruido %.1f, línea %.1f cm" % (n, level, noise, line_w))
    for mid in sorted(ok):
        if not any(m["id"] == mid for m in ms.MISSIONS if m["level"] >= level):
            continue
        print("  M%-3d %-22s %5.0f %%" % (mid, names[mid], 100.0 * ok[mid] / n))
    totals.sort()
    print("  puntos: mediana %d, mín %d, máx %d; caídas %d" %
          (totals[len(totals) // 2], totals[0], totals[-1], falls))
    return ok, totals


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--level", type=int, default=1, help="nivel de inicio (1..4)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--noise", type=float, default=0.0, help="0 = ideal, 1 = ruido realista")
    ap.add_argument("--line-w", type=float, default=2.0, help="ancho de la línea en cm (1..3)")
    ap.add_argument("--set", action="append", help="parámetro del programa clave=valor (cfg.P)")
    ap.add_argument("--true", action="append", help="medida real del robot distinta: clave=valor")
    ap.add_argument("--budget", type=float, default=None, help="segundos para el autónomo")
    ap.add_argument("--ramp-deg", type=float, default=None,
                    help="inclinación de las rampas (reglamento: hasta 40°; plano: ~22°)")
    ap.add_argument("--runs", type=int, default=1, help="> 1 = Monte Carlo (sin reporte HTML)")
    ap.add_argument("--out", default=str(ROOT / "build" / "sim" / "rover_sim.html"))
    ap.add_argument("--no-open", action="store_true")
    a = ap.parse_args(argv)
    prog = parse_kv(a.set)
    true = parse_kv(a.true)
    if a.runs > 1:
        monte_carlo(a.runs, a.level, a.noise, a.line_w, prog, true, a.seed, a.ramp_deg, a.budget)
        return 0
    res = run_once(a.level, a.seed, a.noise, a.line_w, prog, true, a.budget, ramp_deg=a.ramp_deg)
    print_report(res)
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    viewer.write_html(res, out)
    print("Reproducción: %s" % out)
    if not a.no_open and sys.platform == "darwin":
        os.system("open '%s'" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
