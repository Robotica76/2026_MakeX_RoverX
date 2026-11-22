# Rover X 2026: capa de hardware (HAL).
#
# ÚNICO lugar que llama a la API de MicroPython del mBot2 (cyberpi, mbot2, mbuild, gamepad).
# El resto del programa usa estas funciones con convenciones fijas:
#   - drive(izq, der): potencia -100..100, positivo = adelante en ambas orugas
#   - heading(): grados acumulados, positivo = giro a la DERECHA
#   - pitch(): grados, positivo = nariz ARRIBA
#   - line_offset(): -100..100, negativo = la línea está a la IZQUIERDA del centro
# Referencia de la API: docs/api_mbot2.md

import time
import cyberpi
import mbot2
import mbuild
import config as cfg

# El control Bluetooth es opcional (en PC o sin dongle no existe el módulo).
# Ojo: no asignar "gamepad = None"; el bundler renombra las variables globales.
try:
    import gamepad
    _HAS_PAD = True
except ImportError:
    _HAS_PAD = False

PROBES = ("L2", "L1", "R1", "R2")

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
    a = mbot2.EM_get_angle("EM1")
    b = mbot2.EM_get_angle("EM2")
    return (abs(a) + abs(b)) / 2


def reset_odometer():
    global _odo_zero
    _odo_zero = _encoder_deg()


def odometer_cm():
    return (_encoder_deg() - _odo_zero) * cfg.CM_PER_DEG


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
def line_offset():
    return cfg.LINE_SIGN * mbuild.quad_rgb_sensor.get_offset_track(cfg.LINE_SENSOR)


def line_bits():
    """Dict sonda -> True si ve línea negra."""
    sta = mbuild.quad_rgb_sensor.get_line_sta("all", cfg.LINE_SENSOR)
    out = {}
    for p in PROBES:
        out[p] = (sta & cfg.LINE_BIT[p]) != 0
    return out


def probe_rgb(probe):
    q = mbuild.quad_rgb_sensor
    return (q.get_red(probe), q.get_green(probe), q.get_blue(probe))


def probe_color_builtin(probe):
    return mbuild.quad_rgb_sensor.get_color_sta(probe)


# ---------------------------------------------------------------- ultrasónico
def distance_cm():
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
