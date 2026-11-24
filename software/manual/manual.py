## @file manual.py
#  @brief Control manual con el Bluetooth Controller de Makeblock (misiones de alianza).
#  @details Misiones manuales (reglamento): cubos M-A-K-E-X en la barra del nivel 2 (en orden y de
#  frente), activar las rampas (el primer acceso es por la ESCALERA) y 4 cilindros (5 x 15 cm)
#  parados en el área negra.
#
#  Mapeo (ver docs/control_manual.md):
#    Joystick izq. ↕ (Ly)  avanzar / retroceder      Joystick der. ↔ (Rx)  girar
#    Cruceta               movimientos finos lentos (si los joysticks están en el centro)
#    L1 / L2               brazo arriba / abajo (mantener)
#    R1 / R2               garra abrir / cerrar (mantener; entre claw_open y claw_cube - 20°)
#    N1                    modo rápido / preciso (LEDs azul / naranja)
#    N2                    brazo a la altura de la barra (cubos MakeX)
#    N3 (mantener)         escalera: recto con el giroscopio, potencia de escalera
#    N4                    brazo arriba (garra sobre la nariz): viajar, girar cerca de cosas
#    Joystick der. ↕ (Ry)  brazo fino (además de L1/L2)
#
#  Ayudas:
#    - Al avanzar sin girar, el giroscopio SOSTIENE EL RUMBO (rampas, recortes, la oruga
#      izquierda que avanza 2.3 % menos). Al soltar el giro se toma el rumbo nuevo.
#    - hal.drive limita la aceleración mientras va inclinado (no se vuelca en rampas/escalera).
#    - El brazo se mueve por ÁNGULO DEL SERVO (0..90°, tabla medida en config.py): a 0° la garra
#      queda baja y al frente (~8.5 cm), a 90° arriba sobre la nariz.

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


## @brief Zona muerta + curva expo del joystick: más precisión cerca del centro.
#  @param v Valor del eje (-100..100).
#  @return Valor con zona muerta (PAD_DEADZONE) y curva (PAD_EXPO), -100..100.
def shape(v):
    if abs(v) < cfg.PAD_DEADZONE:
        return 0
    x = hal.clamp(v / 100.0, -1, 1)
    y = (1 - cfg.PAD_EXPO) * x + cfg.PAD_EXPO * x * x * x
    return y * 100


## @brief Mezcla arcade -> tanque, escalando para no saturar.
#  @param forward Avance (-100..100).
#  @param turn Giro (-100..100; + = derecha).
#  @param limit Potencia máxima (PAD_FAST o PAD_SLOW).
#  @return (izquierda, derecha) en % de potencia.
def mix(forward, turn, limit):
    left = forward + turn
    right = forward - turn
    m = max(abs(left), abs(right), 100)
    return left * limit / m, right * limit / m


## @brief Detecta el momento en que se presiona una tecla del gamepad.
#  @param state Dict donde se guarda el estado anterior de cada tecla.
#  @param key Nombre de la tecla.
#  @return True solo en el ciclo en que se presiona.
def _edge(state, key):
    now = hal.pad_key(key)
    was = state.get(key, False)
    state[key] = now
    return now and not was


## @brief Corrección de giro para ir recto con el giroscopio mientras no se pide girar.
#  @details Al empezar a avanzar sin girar guarda el rumbo; al pedir un giro lo suelta y al
#  terminar de girar toma el rumbo nuevo.
#  @param f Avance pedido.
#  @param t Giro pedido.
#  @return Giro a aplicar (el pedido, o la corrección para sostener el rumbo, ±25).
def _heading_hold(f, t):
    global hold
    if t != 0 or f == 0:
        hold = None
        return t
    if hold is None:
        hold = hal.heading()
        return 0
    err = hold - hal.heading()
    return hal.clamp(cfg.P["head_kp"] * err, -25, 25)


CLAW_SQUEEZE = 20.0                         # grados que se puede cerrar más allá de claw_cube


## @brief Sentido de apertura del servo de la garra.
#  @return +1 si abrir es subir el ángulo (claw_open >= claw_cube), -1 si no.
def _claw_dir():
    return 1 if cfg.P["claw_open"] >= cfg.P["claw_cube"] else -1


## @brief Limita el ángulo de la garra a su recorrido útil.
#  @details De claw_open (abierta) a claw_cube + CLAW_SQUEEZE (apretando): abrir o cerrar de más
#  fuerza el servo contra el tope.
#  @param deg Ángulo pedido.
#  @return Ángulo limitado.
def _claw_limit(deg):
    shut = cfg.P["claw_cube"] - _claw_dir() * CLAW_SQUEEZE
    return hal.clamp(deg, min(cfg.P["claw_open"], shut), max(cfg.P["claw_open"], shut))


## @brief Fija el ángulo del servo del brazo (0..90°).
#  @param deg Ángulo pedido.
def _arm(deg):
    global arm_deg
    arm_deg = hal.clamp(deg, 0, 90)


## @brief Un ciclo del control manual (cada DT segundos).
#  @details Lee el gamepad: N1 modo rápido/preciso; L1/L2 y Ry brazo; N2 brazo a la barra; N4
#  brazo arriba; R1/R2 garra; N3 (mantener) escalera recto con el giroscopio; Ly/Rx manejo con
#  sostén de rumbo; cruceta movimientos finos. Mapeo completo en docs/control_manual.md.
#  @param state Dict de estado de las teclas (para _edge).
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
        claw = _claw_limit(claw + _claw_dir() * CLAW_RATE * DT)
    elif hal.pad_key("R2"):
        claw = _claw_limit(claw - _claw_dir() * CLAW_RATE * DT)
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


## @brief Texto de estado para la pantalla.
#  @return Modo (RAPIDO/PRECISO), ángulo del brazo y altura de la garra.
def status():
    h, _ = geo.arm_at(arm_deg)
    return "%s brazo %d (%.0f cm)" % ("RAPIDO" if fast else "PRECISO", arm_deg, h)


## @brief Bucle de control manual.
#  @param duration_s Segundos máximos (None = sin límite).
#  @details Sale al pasar duration_s o con A (parar) en el CyberPi; al salir detiene el robot.
def loop(duration_s=None):
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
