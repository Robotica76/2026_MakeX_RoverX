## @file hal.py
#  @brief Capa de hardware (HAL): ÚNICO lugar que llama a la API del mBot2.
#  @details Usa cyberpi, mbot2, mbuild y gamepad (opcional). El resto del programa usa estas
#  funciones con convenciones fijas:
#    - drive(izq, der): potencia -100..100, positivo = adelante en ambas orugas.
#    - heading(): grados acumulados, positivo = giro a la DERECHA.
#    - pitch(): grados, positivo = nariz ARRIBA.
#    - line_offset(): -100..100, negativo = la línea está a la IZQUIERDA del centro.
#    - darkness(): por sonda, 0 = blanco calibrado, 1 = negro calibrado.
#  Referencia de la API: docs/api_mbot2.md. En la PC (pruebas y simulador) se usan módulos
#  falsos con la misma interfaz (tests/fakes).

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


## @brief Limita un valor a un rango.
#  @param v Valor a limitar.
#  @param lo Mínimo.
#  @param hi Máximo.
#  @return v recortado a [lo, hi].
def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


## @brief Inicializa el hardware al arrancar un programa.
#  @details Detiene los motores, pone en cero el rumbo y el odómetro, enciende la luz blanca del
#  Quad RGB (la usan las sondas) y apaga los LEDs del CyberPi. Borra el aviso de abortar.
def init():
    global _abort
    _abort = False
    stop()
    reset_heading()
    reset_odometer()
    mbuild.quad_rgb_sensor.set_led(color="white")
    cyberpi.led.on(0, 0, 0, id="all")


# ---------------------------------------------------------------- tiempo
## @brief Tiempo desde que se encendió el CyberPi.
#  @return Segundos (float).
def now():
    return cyberpi.timer.get()


_on_wait = []            # funciones que se llaman en cada espera (p. ej. odometry.update)


## @brief Espera sin dejar de actualizar la odometría.
#  @details Antes de dormir llama a las funciones registradas en _on_wait (p. ej.
#  odometry.update), así la posición se sigue integrando durante las esperas.
#  @param s Segundos a esperar.
def wait(s):
    for f in _on_wait:
        f()
    time.sleep(s)


# ---------------------------------------------------------------- motores
_last_cmd = [0.0, 0.0]
_last_t = 0.0
_boost = 0.0             # potencia extra de la asistencia de subida
_last_odo = 0.0


## @brief Control contra volcadura en rampas y escalera.
#  @details Inclinado, un arranque brusco empuja el centro de masa detrás de la punta de las
#  orugas y el robot se voltea hacia atrás: con pitch >= tip_pitch la potencia COMÚN (el avance)
#  solo puede subir tip_slew %/s; bajar siempre está permitido. La diferencia entre orugas (la
#  dirección) queda libre para no frenar las correcciones de rumbo.
#  También hace la asistencia de subida: si con potencia hacia adelante el robot casi no avanza
#  (menos de slope_min_v cm/s) o resbala, suma potencia de a poco (hasta +60).
#  No recorta la potencia en pendientes fuertes: en una rampa de 40° eso dejaba al robot sin
#  fuerza a media subida (simulador). El otro control es el brazo (gripper.climb_pose).
#  @param left Potencia pedida a la oruga izquierda (-100..100).
#  @param right Potencia pedida a la oruga derecha (-100..100).
#  @return (izquierda, derecha) con el límite aplicado.
def _tip_guard(left, right):
    global _last_t
    t = now()
    dt = t - _last_t
    _last_t = t
    # Se limita solo la potencia COMÚN (el avance); la diferencia entre orugas (la dirección)
    # queda libre: limitarla también frenaba las correcciones de rumbo y en la escalera una oruga
    # se salía de su tramo (lo mostró el simulador).
    global _boost, _last_odo
    mean = (left + right) / 2.0
    diff = (left - right) / 2.0
    prev = (_last_cmd[0] + _last_cmd[1]) / 2.0
    odo = odo_abs()
    v = (odo - _last_odo) / dt if dt > 0.001 else 0.0
    _last_odo = odo
    if pitch() >= cfg.P["tip_pitch"] and mean > 0:
        # asistencia de subida: si con potencia hacia adelante el robot casi no avanza o se
        # resbala hacia atrás, la potencia sube de a poco (no se queda parado en la pendiente)
        if v < cfg.P["slope_min_v"]:
            _boost = clamp(_boost + cfg.P["slope_boost"] * clamp(dt, 0.0, 0.2), 0, 60)
        else:
            _boost = clamp(_boost - cfg.P["slope_boost"] * clamp(dt, 0.0, 0.2), 0, 60)
        mean = mean + _boost
    else:
        _boost = 0.0
    if abs(pitch()) >= cfg.P["tip_pitch"]:
        step = cfg.P["tip_slew"] * clamp(dt, 0.0, 0.2)
        if abs(mean) > abs(prev) + step:
            mean = prev + step if mean > prev else prev - step
    _last_cmd[0] = mean + diff
    _last_cmd[1] = mean - diff
    return mean + diff, mean - diff


## @brief Mueve las orugas con potencia (convención del programa: + = adelante).
#  @details Pasa por _tip_guard y, si una oruga pasaría de 100 %, baja las dos lo mismo para
#  conservar la diferencia (la corrección de rumbo): recortar solo una dejaba casi sin corrección
#  a potencia máxima y en la escalera una oruga se salía de su tramo (simulador). Aplica los
#  signos de montaje LEFT_SIGN / RIGHT_SIGN.
#  @param left Potencia de la oruga izquierda, -100..100.
#  @param right Potencia de la oruga derecha, -100..100.
def drive(left, right):
    left, right = _tip_guard(left, right)
    # prioridad a la DIRECCIÓN: si una oruga pasaría de 100 %, se bajan las dos lo mismo y la
    # diferencia (la corrección de rumbo) se conserva. Recortar solo una dejaba casi sin
    # corrección a potencia máxima: en la escalera el robot se desviaba y una oruga se salía de
    # su tramo (lo mostró el simulador).
    over = max(abs(left), abs(right)) - 100
    if over > 0:
        mean = (left + right) / 2.0
        diff = (left - right) / 2.0
        if mean > 0:
            mean -= over
        else:
            mean += over
        left = mean + diff
        right = mean - diff
    left = clamp(left, -100, 100)
    right = clamp(right, -100, 100)
    mbot2.drive_power(cfg.LEFT_SIGN * left, cfg.RIGHT_SIGN * right)


## @brief Detiene las dos orugas (potencia 0).
def stop():
    _last_cmd[0] = 0.0
    _last_cmd[1] = 0.0
    mbot2.drive_power(0, 0)


## @brief Avance del robot en grados de encoder, con signo.
#  @details Promedio de las dos orugas con su signo: un giro en el lugar no suma (antes se usaba
#  abs() y los giros cambiaban la odometría, lo que movía las distancias medidas desde un cruce).
#  @return Grados (adelante +, atrás -).
def _encoder_deg():
    a = cfg.LEFT_SIGN * mbot2.EM_get_angle("EM1")
    b = cfg.RIGHT_SIGN * mbot2.EM_get_angle("EM2")
    return (a + b) / 2


## @brief Avance de cada oruga en cm desde el arranque, con signo.
#  @return (izquierda, derecha) en cm; usa los cm por grado de cada oruga (lr_ratio).
def tracks_cm():
    kl, kr = geo.cm_per_deg_lr()
    return (cfg.LEFT_SIGN * mbot2.EM_get_angle("EM1") * kl,
            cfg.RIGHT_SIGN * mbot2.EM_get_angle("EM2") * kr)


## @brief Pone en cero el odómetro relativo (odometer_cm).
def reset_odometer():
    global _odo_zero
    _odo_zero = _encoder_deg()


## @brief Avance absoluto desde el arranque (no se reinicia).
#  @return cm con signo.
def odo_abs():
    return _encoder_deg() * geo.cm_per_deg()


## @brief Avance desde el último reset_odometer().
#  @return cm con signo.
def odometer_cm():
    return (_encoder_deg() - _odo_zero) * geo.cm_per_deg()


## @brief Mueve un servo.
#  @param port Puerto del servo ("s1" garra, "s2" brazo; ver cfg.SERVO_CLAW / SERVO_ARM).
#  @param angle Ángulo en grados; se limita a 0..180.
def servo(port, angle):
    mbot2.servo_set(clamp(angle, 0, 180), port)


# ---------------------------------------------------------------- giroscopio
## @brief Rumbo del giroscopio sin el cero del programa.
#  @details Aplica HEADING_SIGN y gyro_scale (grados reales por grado leído; TODO calibrar:
#  girar el robot A MANO 360° contra una escuadra y poner 360 / lectura).
#  @return Grados acumulados, + = derecha.
def _raw_heading():
    # gyro_scale: grados reales por grado que reporta el giroscopio (TODO calibrar: girar el
    # robot A MANO 360° contra una escuadra y leer get_rotation("z"); 360 / lectura)
    return cfg.HEADING_SIGN * cyberpi.get_rotation("z") * cfg.P.get("gyro_scale", 1.0)


## @brief Toma el rumbo actual como 0.
def reset_heading():
    global _heading_zero
    _heading_zero = _raw_heading()


## @brief Rumbo del robot.
#  @return Grados acumulados desde reset_heading(); positivo = giro a la DERECHA.
def heading():
    return _raw_heading() - _heading_zero


## @brief Inclinación del robot (cabeceo).
#  @return Grados; positivo = nariz ARRIBA (aplica PITCH_SIGN).
def pitch():
    return cfg.PITCH_SIGN * cyberpi.get_pitch()


# ---------------------------------------------------------------- Quad RGB
# Con línea delgada (10-30 mm) la línea puede quedar ENTRE dos sondas y el modo binario la
# pierde. En line_mode = 1 se usa la luz de cada sonda calibrada contra blanco y negro: una
# sonda medio cubierta da ~0.5 y el centro ponderado sigue siendo correcto.

## @brief Luz reflejada que lee una sonda del Quad RGB.
#  @param probe Nombre de la sonda: "L2", "L1", "R1" o "R2".
#  @return Lectura cruda de luz.
def probe_light(probe):
    return mbuild.quad_rgb_sensor.get_light(probe)


## @brief Oscuridad de cada sonda según la calibración LINE_CAL.
#  @details Con línea delgada (10-30 mm) la línea puede quedar ENTRE dos sondas; una sonda medio
#  cubierta da ~0.5 y el centro ponderado (offset_from) sigue siendo correcto.
#  @return Dict sonda -> 0..1 (0 = blanco calibrado, 1 = negro calibrado).
def darkness():
    out = {}
    for pr in PROBES:
        white, black = cfg.LINE_CAL[pr]
        span = white - black
        if span < 1:
            span = 1
        out[pr] = clamp((white - probe_light(pr)) / span, 0, 1)
    return out


## @brief Posición de la línea a partir de la oscuridad de las sondas (función pura).
#  @details Centro ponderado usando la posición real de cada sonda en cm (no están igual de
#  separadas: 2.4 / 1.25 / 2.4). Antes resta el fondo (la sonda más clara): sobre una zona oscura
#  (azul, verde) todas las sondas suben igual y la línea se perdía en el promedio.
#  @param dk Dict sonda -> oscuridad 0..1 (de darkness()).
#  @return -100..100; negativo = línea a la izquierda; 0 si no hay línea.
def offset_from(dk):
    base = 1.0
    for i in range(4):
        if dk[PROBES[i]] < base:
            base = dk[PROBES[i]]
    # posición real de cada sonda en cm (no están igual de separadas: 2.4 / 1.25 / 2.4)
    pi = cfg.P["probe_pitch"] / 2.0
    po = pi + cfg.P.get("probe_outer", cfg.P["probe_pitch"])
    lat = (-po, -pi, pi, po)
    total = 0
    acc = 0
    for i in range(4):
        d = dk[PROBES[i]] - base
        total += d
        acc += d * lat[i]
    if total < 0.05:
        return 0
    return clamp(acc / total / po * 100, -100, 100)


## @brief Posición de la línea bajo el sensor.
#  @details line_mode 1: analógico calibrado (offset_from); 0: get_offset_track del sensor.
#  @return -100..100; negativo = la línea está a la IZQUIERDA.
def line_offset():
    if cfg.P["line_mode"] == 1:
        return offset_from(darkness())
    return cfg.LINE_SIGN * mbuild.quad_rgb_sensor.get_offset_track(cfg.LINE_SENSOR)


## @brief Posición de la línea y sondas sobre negro con UNA sola lectura.
#  @details Para el lazo de control. Además de L2..R2 devuelve "L2x"/"R2x": las exteriores con
#  umbral cross_th (negro "fuerte"), porque las marcas verdes junto a la línea también son
#  oscuras y adelantaban el cruce (simulador).
#  @return (offset, bits): offset -100..100 y dict sonda -> True si ve línea.
def line_read():
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


## @brief Qué sondas ven línea negra.
#  @return Dict sonda -> True si ve línea (umbral dark_th, o bits de get_line_sta en line_mode 0).
def line_bits():
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


## @brief Color RGB que lee una sonda.
#  @param probe Nombre de la sonda.
#  @return (rojo, verde, azul).
def probe_rgb(probe):
    q = mbuild.quad_rgb_sensor
    return (q.get_red(probe), q.get_green(probe), q.get_blue(probe))


## @brief Nombre del color que reconoce el propio sensor.
#  @param probe Nombre de la sonda.
#  @return Nombre del color según get_color_sta (color_mode 0).
def probe_color_builtin(probe):
    return mbuild.quad_rgb_sensor.get_color_sta(probe)


# ---------------------------------------------------------------- ultrasónico (opcional)
## @brief Distancia del Ultrasónico 2.
#  @return cm, o -1 si el robot no lo lleva (cfg.HAS_US = False, diseño actual).
def distance_cm():
    if not cfg.HAS_US:
        return -1
    return mbuild.ultrasonic2.get(index=cfg.US_SENSOR)


# ---------------------------------------------------------------- CyberPi: botones, pantalla, LEDs
## @brief Lee un botón del CyberPi.
#  @param name "a", "b", "up", "down", "left", "right" o "middle".
#  @return True si está presionado.
def button(name):
    return cyberpi.controller.is_press(name)


## @brief ¿Se pidió parar? (botón A, cfg.ABORT_BUTTON).
#  @details Queda marcado hasta clear_abort(): una pulsación corta basta para detener el
#  autónomo o el control manual.
#  @return True si se presionó A desde el último clear_abort().
def abort_requested():
    global _abort
    if not _abort and cyberpi.controller.is_press(cfg.ABORT_BUTTON):
        _abort = True
    return _abort


## @brief Borra el aviso de parar (después de atenderlo).
def clear_abort():
    global _abort
    _abort = False


## @brief Escribe un renglón en la pantalla del CyberPi.
#  @param text Texto a mostrar.
#  @param line Renglón (0 arriba; 18 px por renglón).
#  @param size Tamaño de letra.
def show(text, line=0, size=16):
    cyberpi.display.show_label(text, size, 0, line * 18, index=line)


## @brief Enciende los LEDs del CyberPi de un color.
#  @param r Rojo 0..255.
#  @param g Verde 0..255.
#  @param b Azul 0..255.
def leds(r, g, b):
    cyberpi.led.on(r, g, b, id="all")


# ---------------------------------------------------------------- control Bluetooth
## @brief ¿Hay módulo de gamepad (Bluetooth Controller)?
#  @return True si se pudo importar gamepad.
def pad_ok():
    return _HAS_PAD


## @brief Lee un botón del gamepad.
#  @param name Nombre del botón (p. ej. "N1", "R1", "Up"; ver docs/control_manual.md).
#  @return True si está presionado (False si no hay gamepad).
def pad_key(name):
    if not _HAS_PAD:
        return False
    return gamepad.is_key_pressed(name)


## @brief Lee un eje de los joysticks del gamepad.
#  @param name "Lx", "Ly", "Rx" o "Ry".
#  @return Valor del eje (0 si no hay gamepad).
def pad_axis(name):
    if not _HAS_PAD:
        return 0
    return gamepad.get_joystick(name)
