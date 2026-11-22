# Rover X 2026: control manual con el Bluetooth Controller de Makeblock (misiones de alianza).
#
# Mapeo (ver docs/control_manual.md):
#   Joystick izq. Y  -> avanzar / retroceder      Joystick der. X -> girar
#   L1 / L2          -> brazo arriba / abajo      R1 / R2         -> garra abrir / cerrar
#   N1               -> modo rápido / preciso     N2              -> brazo a altura de barra
#   N3               -> subir escalera (asistido, mantener)        N4 -> brazo a posición de viaje
#   Up/Down/Left/Right -> movimientos finos lentos

import hal
import config as cfg
import geometry as geo

claw = cfg.P["claw_open"]
arm = cfg.P["h_carry"]   # altura de la garra en cm (se convierte a ángulo con geometry)
fast = True


def shape(v):
    """Zona muerta + curva expo: más precisión cerca del centro del joystick."""
    if abs(v) < cfg.PAD_DEADZONE:
        return 0
    x = hal.clamp(v / 100.0, -1, 1)
    y = (1 - cfg.PAD_EXPO) * x + cfg.PAD_EXPO * x * x * x
    return y * 100


def mix(forward, turn, limit):
    """Arcade -> tanque, escalando para no saturar."""
    left = forward + turn
    right = forward - turn
    m = max(abs(left), abs(right), 100)
    return left * limit / m, right * limit / m


def _edge(state, key):
    """True solo en el momento en que se presiona la tecla."""
    now = hal.pad_key(key)
    was = state.get(key, False)
    state[key] = now
    return now and not was


def step(state):
    global claw, arm, fast
    if _edge(state, "N1"):
        fast = not fast
        if fast:
            hal.leds(0, 0, 120)
        else:
            hal.leds(120, 60, 0)
    limit = cfg.PAD_FAST if fast else cfg.PAD_SLOW

    if hal.pad_key("N3"):
        # escalera: potencia máxima recta mientras se mantiene N3
        hal.drive(cfg.P["v_stairs"], cfg.P["v_stairs"])
        return

    f = cfg.PAD_Y_SIGN * shape(hal.pad_axis("Ly"))
    t = cfg.PAD_TURN_GAIN * shape(hal.pad_axis("Rx"))
    if f == 0 and t == 0:
        slow = cfg.PAD_SLOW * 0.6
        if hal.pad_key("Up"):
            f = slow
        elif hal.pad_key("Down"):
            f = -slow
        elif hal.pad_key("Left"):
            t = -slow
        elif hal.pad_key("Right"):
            t = slow
        limit = cfg.PAD_SLOW
    left, right = mix(f, t, limit)
    hal.drive(left, right)

    if hal.pad_key("L1"):
        arm = hal.clamp(arm + 0.3, 0, 40)
    elif hal.pad_key("L2"):
        arm = hal.clamp(arm - 0.3, 0, 40)
    if _edge(state, "N2"):
        arm = cfg.P["h_bar"]
    if _edge(state, "N4"):
        arm = cfg.P["h_carry"]
    if hal.pad_key("R1"):
        claw = hal.clamp(claw + 3, 0, 180)
    elif hal.pad_key("R2"):
        claw = hal.clamp(claw - 3, 0, 180)
    hal.servo(cfg.SERVO_ARM, geo.arm_servo(arm))
    hal.servo(cfg.SERVO_CLAW, claw)


def loop(duration_s=None):
    """Bucle de control manual. Sale al pasar duration_s o con B en el CyberPi."""
    hal.clear_abort()
    hal.show("MANUAL", line=0, size=24)
    if not hal.pad_ok():
        hal.show("Sin modulo gamepad", line=2)
    state = {}
    t0 = hal.now()
    while duration_s is None or hal.now() - t0 < duration_s:
        if hal.abort_requested():
            break
        step(state)
        hal.wait(0.02)
    hal.stop()

