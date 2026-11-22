# Rover X 2026: capa de hardware (HAL).
#
# ÚNICO lugar que llama a la API de MicroPython del mBot2 (cyberpi, mbot2, mbuild, gamepad).
# El resto del programa usa estas funciones con convenciones fijas:
#   - drive(izq, der): potencia -100..100, positivo = adelante en ambas orugas
#   - heading(): grados acumulados, positivo = giro a la DERECHA
#   - pitch(): grados, positivo = nariz ARRIBA
#   - line_offset(): -100..100, negativo = la línea está a la IZQUIERDA del centro
#   - darkness(): por sonda, 0 = blanco calibrado, 1 = negro calibrado
# Referencia de la API: docs/api_mbot2.md

import time
import cyberpi
import mbot2
import mbuild
import config as cfg
import geometry as geo

# El control Bluetooth es opcional (en PC o sin dongle no existe el módulo).
# Ojo: no asignar "gamepad = None"; el bundler renombra las variables globales.
try:
    import gamepad
    _HAS_PAD = True
except ImportError:
    _HAS_PAD = False

PROBES = cfg.PROBES

_heading_zero = 0
_odo_zero = 0
_abort = False


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def init():
    global _abort
    _abort = False
    stop()
    reset_heading()
    reset_odometer()
    mbuild.quad_rgb_sensor.set_led(color="white")
    cyberpi.led.on(0, 0, 0, id="all")


# ---------------------------------------------------------------- tiempo
def now():
    return cyberpi.timer.get()


def wait(s):
    time.sleep(s)


# ---------------------------------------------------------------- motores
def drive(left, right):
    left = clamp(left, -100, 100)
    right = clamp(right, -100, 100)
    mbot2.drive_power(cfg.LEFT_SIGN * left, cfg.RIGHT_SIGN * right)


def stop():
    mbot2.drive_power(0, 0)


def _encoder_deg():
    """Avance del robot en grados de encoder, CON signo (adelante +, atrás -).

    Es el promedio de las dos orugas con su signo, así que un giro en el lugar no suma
    (antes se usaba abs() y los giros cambiaban la odometría, lo que movía las distancias
    medidas desde un cruce o una tarjeta)."""
    a = cfg.LEFT_SIGN * mbot2.EM_get_angle("EM1")
    b = cfg.RIGHT_SIGN * mbot2.EM_get_angle("EM2")
    return (a + b) / 2


def reset_odometer():
    global _odo_zero
    _odo_zero = _encoder_deg()


def odo_abs():
    """Avance absoluto (cm desde el arranque, con signo); no se reinicia."""
    return _encoder_deg() * geo.cm_per_deg()


def odometer_cm():
    """Avance (cm, con signo) desde el último reset_odometer()."""
    return (_encoder_deg() - _odo_zero) * geo.cm_per_deg()


def servo(port, angle):
    mbot2.servo_set(clamp(angle, 0, 180), port)


# ---------------------------------------------------------------- giroscopio
def _raw_heading():
    return cfg.HEADING_SIGN * cyberpi.get_rotation("z")


def reset_heading():
    global _heading_zero
    _heading_zero = _raw_heading()


def heading():
    return _raw_heading() - _heading_zero


def pitch():
    return cfg.PITCH_SIGN * cyberpi.get_pitch()


# ---------------------------------------------------------------- Quad RGB
# Con línea delgada (10-30 mm) la línea puede quedar ENTRE dos sondas y el modo binario la
# pierde. En line_mode = 1 se usa la luz de cada sonda calibrada contra blanco y negro: una
# sonda medio cubierta da ~0.5 y el centro ponderado sigue siendo correcto.

def probe_light(probe):
    return mbuild.quad_rgb_sensor.get_light(probe)


def darkness():
    """Dict sonda -> 0..1 (0 = blanco, 1 = negro) según la calibración LINE_CAL."""
    out = {}
    for pr in PROBES:
        white, black = cfg.LINE_CAL[pr]
        span = white - black
        if span < 1:
            span = 1
        out[pr] = clamp((white - probe_light(pr)) / span, 0, 1)
    return out


def offset_from(dk):
    """Centro ponderado de la oscuridad -> -100..100 (función pura, probada en PC).

    Se resta el fondo (la sonda más clara): sobre una zona oscura (azul, verde) todas las sondas
    suben igual y la línea se perdía en el promedio (la curva del nivel 3 empieza dentro de la
    zona azul; lo mostró el simulador)."""
    base = 1.0
    for i in range(4):
        if dk[PROBES[i]] < base:
            base = dk[PROBES[i]]
    total = 0
    acc = 0
    for i in range(4):
        d = dk[PROBES[i]] - base
        total += d
        acc += d * (i - 1.5)
    if total < 0.05:
        return 0
    return clamp(acc / total / 1.5 * 100, -100, 100)


def line_offset():
    if cfg.P["line_mode"] == 1:
        return offset_from(darkness())
    return cfg.LINE_SIGN * mbuild.quad_rgb_sensor.get_offset_track(cfg.LINE_SENSOR)


def line_read():
    """(offset, bits) con UNA sola lectura de las sondas (para el lazo de control)."""
    if cfg.P["line_mode"] == 1:
        dk = darkness()
        th = cfg.P["dark_th"]
        bits = {}
        for pr in PROBES:
            bits[pr] = dk[pr] >= th
        # para los cruces, las exteriores piden negro "fuerte": las marcas verdes junto a la
        # línea también son oscuras y adelantaban el cruce (lo mostró el simulador)
        bits["L2x"] = dk["L2"] >= cfg.P["cross_th"]
        bits["R2x"] = dk["R2"] >= cfg.P["cross_th"]
        return offset_from(dk), bits
    return line_offset(), line_bits()


def line_bits():
    """Dict sonda -> True si ve línea negra."""
    out = {}
    if cfg.P["line_mode"] == 1:
        dk = darkness()
        th = cfg.P["dark_th"]
        for pr in PROBES:
            out[pr] = dk[pr] >= th
        return out
    sta = mbuild.quad_rgb_sensor.get_line_sta("all", cfg.LINE_SENSOR)
    for pr in PROBES:
        out[pr] = (sta & cfg.LINE_BIT[pr]) != 0
    return out


def probe_rgb(probe):
    q = mbuild.quad_rgb_sensor
    return (q.get_red(probe), q.get_green(probe), q.get_blue(probe))


def probe_color_builtin(probe):
    return mbuild.quad_rgb_sensor.get_color_sta(probe)


# ---------------------------------------------------------------- ultrasónico (opcional)
def distance_cm():
    """Distancia del Ultrasónico 2, o -1 si el robot no lo lleva (HAS_US = False)."""
    if not cfg.HAS_US:
        return -1
    return mbuild.ultrasonic2.get(index=cfg.US_SENSOR)


# ---------------------------------------------------------------- CyberPi: botones, pantalla, LEDs
def button(name):
    return cyberpi.controller.is_press(name)


def abort_requested():
    """Botón B del CyberPi = abortar el autónomo (para reinicios)."""
    global _abort
    if not _abort and cyberpi.controller.is_press("b"):
        _abort = True
    return _abort


def clear_abort():
    global _abort
    _abort = False


def show(text, line=0, size=16):
    cyberpi.display.show_label(text, size, 0, line * 18, index=line)


def leds(r, g, b):
    cyberpi.led.on(r, g, b, id="all")


# ---------------------------------------------------------------- control Bluetooth
def pad_ok():
    return _HAS_PAD


def pad_key(name):
    if not _HAS_PAD:
        return False
    return gamepad.is_key_pressed(name)


def pad_axis(name):
    if not _HAS_PAD:
        return 0
    return gamepad.get_joystick(name)
