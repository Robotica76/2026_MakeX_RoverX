"""Mundo simulado mínimo para correr el código del robot en PC.

Los módulos falsos cyberpi / mbot2 / mbuild / gamepad leen y escriben aquí. La cinemática es
simple: la potencia de cada oruga integra el ángulo del encoder y el rumbo; el reloj avanza
solo cuando el programa llama a time.sleep (hal.wait). Las pruebas pueden poner funciones
en `sensors` para simular la pista.
"""

DEG_PER_POWER_S = 10.0      # grados de encoder por (% potencia * s)
TRACK_TURN = 0.6            # grados de rumbo por (diferencia de potencia * s)

t = 0.0
em = [0.0, 0.0]             # potencia aplicada EM1, EM2 (convención mbot2)
enc = [0.0, 0.0]            # ángulo de encoder EM1, EM2
rot_z = 0.0                 # rotación z del CyberPi
servos = {}
pressed = set()             # botones del CyberPi
pad = set()                 # teclas del gamepad
axes = {}
labels = {}
leds = (0, 0, 0)

# funciones de sensores (las pruebas las reemplazan)
sensors = {
    "offset": lambda: 0,
    "line_sta": lambda: 6,
    "color": lambda probe: "white",
    "rgb": lambda probe: (230, 230, 230),
    "distance": lambda: 300,
    "pitch": lambda: 0,
    "light": lambda probe: 10 if probe in ("L1", "R1") else 90,   # línea bajo el centro
}


def reset():
    global t, em, enc, rot_z, servos, pressed, pad, axes, labels, leds
    t = 0.0
    em = [0.0, 0.0]
    enc = [0.0, 0.0]
    rot_z = 0.0
    servos = {}
    pressed = set()
    pad = set()
    axes = {}
    labels = {}
    leds = (0, 0, 0)
    sensors.update({
        "offset": lambda: 0,
        "line_sta": lambda: 6,
        "color": lambda probe: "white",
        "rgb": lambda probe: (230, 230, 230),
        "distance": lambda: 300,
        "pitch": lambda: 0,
        "light": lambda probe: 10 if probe in ("L1", "R1") else 90,
    })


def advance(dt):
    global t, rot_z
    t += dt
    left = em[0]
    right = -em[1]                      # EM2 invertido: adelante = negativo
    enc[0] += em[0] * DEG_PER_POWER_S * dt
    enc[1] += em[1] * DEG_PER_POWER_S * dt
    # girar a la derecha (izq > der) hace bajar rot_z en el CyberPi (HEADING_SIGN = -1)
    rot_z -= (left - right) * TRACK_TURN * dt
