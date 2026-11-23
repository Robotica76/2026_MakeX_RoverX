# Rover X 2026: control manual con el Bluetooth Controller de Makeblock (misiones de alianza).
#
# Misiones manuales (reglamento): cubos M-A-K-E-X en la barra del nivel 2 (en orden y de
# frente), activar las rampas (el primer acceso es por la ESCALERA) y 4 cilindros (5 x 15 cm)
# parados en el área negra.
#
# Mapeo (ver docs/control_manual.md):
#   Joystick izq. ↕ (Ly)  avanzar / retroceder      Joystick der. ↔ (Rx)  girar
#   Cruceta               movimientos finos lentos (si los joysticks están en el centro)
#   L1 / L2               brazo arriba / abajo (mantener)
#   R1 / R2               garra abrir / cerrar (mantener)
#   N1                    modo rápido / preciso (LEDs azul / naranja)
#   N2                    brazo a la altura de la barra (cubos MakeX)
#   N3 (mantener)         escalera: recto con el giroscopio, potencia de escalera
#   N4                    brazo arriba (garra sobre la nariz): viajar, girar cerca de cosas
#   Joystick der. ↕ (Ry)  brazo fino (además de L1/L2)
#
# Ayudas:
#   - Al avanzar sin girar, el giroscopio SOSTIENE EL RUMBO (rampas, recortes, la oruga
#     izquierda que avanza 2.3 % menos). Al soltar el giro se toma el rumbo nuevo.
#   - hal.drive limita la aceleración mientras va inclinado (no se vuelca en rampas/escalera).
#   - El brazo se mueve por ÁNGULO DEL SERVO (0..90°, tabla medida en config.py): a 0° la garra
#     queda baja y al frente (~8.5 cm), a 90° arriba sobre la nariz.

import hal
import config as cfg
import geometry as geo

claw = cfg.P["claw_open"]
arm_deg = geo.arm_servo(cfg.P["h_top"])     # ángulo del servo del brazo
fast = True
hold = None                                 # rumbo a sostener al avanzar recto (o None)
ARM_RATE = 60.0                             # grados de servo por segundo con L1/L2
CLAW_RATE = 150.0                           # grados de servo por segundo con R1/R2
DT = 0.02


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


def _heading_hold(f, t):
    """Corrección de giro para ir recto con el giroscopio mientras no se pide girar."""
    global hold
    if t != 0 or f == 0:
        hold = None
        return t
    if hold is None:
        hold = hal.heading()
        return 0
    err = hold - hal.heading()
    return hal.clamp(cfg.P["head_kp"] * err, -25, 25)


def _arm(deg):
    global arm_deg
    arm_deg = hal.clamp(deg, 0, 90)


def step(state):
    global claw, fast, hold
    if _edge(state, "N1"):
        fast = not fast
        if fast:
            hal.leds(0, 0, 120)
        else:
            hal.leds(120, 60, 0)
    limit = cfg.PAD_FAST if fast else cfg.PAD_SLOW

    # ---- brazo y garra (también mientras sube la escalera)
    if hal.pad_key("L1"):
        _arm(arm_deg + ARM_RATE * DT)
    elif hal.pad_key("L2"):
        _arm(arm_deg - ARM_RATE * DT)
    ry = shape(hal.pad_axis("Ry"))
    if ry:
        _arm(arm_deg + ry / 100.0 * ARM_RATE * DT)
    if _edge(state, "N2"):
        _arm(geo.arm_servo(cfg.P["h_bar"]))
    if _edge(state, "N4"):
        _arm(geo.arm_servo(cfg.P["h_top"]))
    if hal.pad_key("R1"):
        claw = hal.clamp(claw + CLAW_RATE * DT, 0, 180)
    elif hal.pad_key("R2"):
        claw = hal.clamp(claw - CLAW_RATE * DT, 0, 180)
    hal.servo(cfg.SERVO_ARM, arm_deg)
    hal.servo(cfg.SERVO_CLAW, claw)

    # ---- escalera: recto con el giroscopio mientras se mantiene N3
    if hal.pad_key("N3"):
        if hold is None:
            hold = hal.heading()
        corr = hal.clamp(cfg.P["head_kp"] * (hold - hal.heading()), -25, 25)
        v = cfg.P["v_stairs"]
        hal.drive(v + corr, v - corr)
        return

    # ---- manejo
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
    t = _heading_hold(f, t)
    left, right = mix(f, t, limit)
    hal.drive(left, right)


def status():
    h, _ = geo.arm_at(arm_deg)
    return "%s brazo %d (%.0f cm)" % ("RAPIDO" if fast else "PRECISO", arm_deg, h)


def loop(duration_s=None):
    """Bucle de control manual. Sale al pasar duration_s o con A (abortar) en el CyberPi."""
    global hold
    hold = None
    hal.clear_abort()
    hal.show("MANUAL", line=0, size=24)
    if not hal.pad_ok():
        hal.show("Sin modulo gamepad", line=2)
    state = {}
    t0 = hal.now()
    shown = 0
    while duration_s is None or hal.now() - t0 < duration_s:
        if hal.abort_requested():
            break
        step(state)
        shown += 1
        if shown % 10 == 0:
            hal.show(status(), line=1)
        hal.wait(DT)
    hal.stop()
