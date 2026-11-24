# rover_x_manual.py bundled for mBlock with tools/scripts/bundle_for_mblock.py (generated:
# edit the source files and bundle again). Modules: config.py, geometry.py, hal.py, manual.py, gripper.py, colors.py, motion.py, settings.py
import time
import cyberpi
import mbot2
import mbuild
import math
import json

# ---- software/core/config.py ----
config_LINE_SENSOR = 1
config_HAS_US = False
config_US_SENSOR = 1
config_SERVO_CLAW = 's1'
config_SERVO_ARM = 's2'
config_PROBES = ('L2', 'L1', 'R1', 'R2')
config_PARAMS_FILE = 'rover_params.json'
config_LEFT_SIGN = 1
config_RIGHT_SIGN = -1
config_HEADING_SIGN = -1
config_PITCH_SIGN = 1
config_LINE_SIGN = 1
config_PAD_Y_SIGN = 1
config_LINE_BIT = {'L2': 8, 'L1': 4, 'R1': 2, 'R2': 1}
config_LOOP_DT = 0.01
config_P = {'ref_rot_fwd': 4.5, 'track_c': 12.1, 'track_d': 4.85, 'nose_deg': 45.0, 'wheel_d': 3.95, 'belt_t': 0.4, 'belt_w': 3.0, 'robot_w': 21.35, 'sensor_fwd': 9.7, 'probe_pitch': 1.25, 'probe_outer': 2.4, 'card_probe_fwd': 9.7, 'sprocket_d': 4.18, 'gyro_scale': 1.0, 'lr_ratio': 0.977, 'skid': 1.3, 'slip_tol': 0.05, 'track_max': 15.0, 'track_k': 4.0, 'slip': 1.0, 'arm_h0': 8.5, 'arm_x0': 8.0, 'arm_h45': 14.5, 'arm_x45': 6.0, 'arm_h90': 23.0, 'arm_x90': 0.0, 'arm_pivot_fwd': 10.0, 'arm_pivot_h': 12.2, 'arm_ground': 5.0, 'arm_ground_dx': 0.7, 'arm_crank': 7.2, 'arm_rocker': 5.75, 'arm_coupler': 7.0, 'arm_claw_u': 1.9, 'arm_claw_v': 7.9, 'claw_tip': 3.0, 'h_floor': 8.5, 'h_pedestal': 12.5, 'h_carry': 17.5, 'h_bar': 12.0, 'h_prism': 8.5, 'claw_open': 90, 'claw_cube': 35, 'claw_ball': 45, 'com_fwd': -1.0, 'com_h': 6.5, 'arm_mass': 0.15, 'arm_w': 4.9, 'arm_lat': -0.75, 'claw_open_w': 9.0, 'line_mode': 1, 'dark_th': 0.45, 'cross_win': 4.0, 'cross_th': 0.8, 'kp': 0.45, 'kd': 1.5, 'ki': 0.0, 'lost_s': 0.6, 'green': 0, 'v_line': 45, 'v_slow': 25, 'v_straight': 45, 'v_turn_max': 45, 'v_turn_min': 18, 'v_ramp': 70, 'v_stairs': 100, 'v_push': 30, 'turn_kp': 1.0, 'turn_tol': 2.5, 'head_kp': 1.2, 'pitch_ramp': 10, 'pitch_flat': 4, 'ramp_margin': 3.0, 'tip_pitch': 8, 'tip_slew': 120, 'slope_min_v': 4.0, 'slope_boost': 60, 'h_top': 23.0, 'h_climb': 10.5, 'ped1_from_cross': 12.4, 'ped2_from_cross': 11.1, 'ball_ped_dist': 23.6, 'tower_from_edge': 7.5, 'tower_size': 10.0, 'side_margin': 2.0, 'bar_side': 1, 'bar_reach': 8.2, 'color_mode': 0}
config_TUNE = [('Linea', 'kp', 0.05, 0, 3), ('Linea', 'kd', 0.1, 0, 10), ('Linea', 'v_line', 5, 10, 100), ('Linea', 'dark_th', 0.05, 0.1, 0.9), ('Linea', 'cross_win', 0.5, 1, 15), ('Linea', 'cross_th', 0.05, 0.3, 1), ('Linea', 'line_mode', 1, 0, 1), ('Linea', 'green', 1, 0, 1), ('Geometria', 'ref_rot_fwd', 0.5, -20, 20), ('Geometria', 'sensor_fwd', 0.5, -10, 40), ('Geometria', 'card_probe_fwd', 0.5, -10, 40), ('Geometria', 'track_c', 0.5, 5, 40), ('Geometria', 'belt_w', 0.5, 1, 10), ('Geometria', 'robot_w', 0.5, 10, 40), ('Geometria', 'sprocket_d', 0.02, 2, 10), ('Geometria', 'lr_ratio', 0.005, 0.9, 1.1), ('Geometria', 'gyro_scale', 0.01, 0.5, 2), ('Geometria', 'slip', 0.02, 0.5, 1.5), ('Brazo', 'arm_h0', 0.5, 0, 40), ('Brazo', 'arm_x0', 0.5, -20, 30), ('Brazo', 'arm_h45', 0.5, 0, 40), ('Brazo', 'arm_x45', 0.5, -20, 30), ('Brazo', 'arm_h90', 0.5, 0, 40), ('Brazo', 'arm_x90', 0.5, -20, 30), ('Brazo', 'h_pedestal', 0.5, 0, 30), ('Brazo', 'h_carry', 0.5, 0, 30), ('Brazo', 'h_bar', 0.5, 0, 30), ('Brazo', 'h_prism', 0.5, 0, 30), ('Brazo', 'h_floor', 0.5, 0, 30), ('Garra', 'claw_open', 2, 0, 180), ('Garra', 'claw_cube', 2, 0, 180), ('Garra', 'claw_ball', 2, 0, 180), ('Movimiento', 'v_slow', 5, 10, 100), ('Movimiento', 'v_ramp', 5, 10, 100), ('Movimiento', 'v_stairs', 5, 10, 100), ('Movimiento', 'v_turn_min', 1, 5, 60), ('Movimiento', 'turn_kp', 0.1, 0.1, 5), ('Movimiento', 'tip_slew', 10, 20, 400), ('Brazo', 'h_climb', 0.5, 0, 30), ('Movimiento', 'head_kp', 0.1, 0, 5), ('Pista', 'ped1_from_cross', 0.5, 0, 60), ('Pista', 'ped2_from_cross', 0.5, 0, 60), ('Pista', 'ball_ped_dist', 0.5, 0, 60), ('Pista', 'tower_from_edge', 0.5, 0, 100), ('Pista', 'side_margin', 0.5, 0, 15), ('Pista', 'bar_side', 2, -1, 1), ('Pista', 'bar_reach', 0.5, 0, 30)]
config_LINE_CAL = {'L2': [90, 10], 'L1': [90, 10], 'R1': [90, 10], 'R2': [90, 10]}
config_COLOR_REF = {'black': (20, 20, 20), 'white': (230, 230, 230), 'red': (200, 40, 40), 'yellow': (220, 200, 50), 'green': (40, 160, 70), 'blue': (40, 90, 200), 'cyan': (60, 180, 200), 'purple': (140, 60, 170)}
config_COLOR_CONFIRM = 3
config_CARD_PROBE = 'R1'
config_CARD_COUNT = 6
config_CARD_SCAN_MAX_CM = 120
config_ZONE_LEVEL = {'red': 1, 'yellow': 2, 'blue': 3, 'green': 4}
config_GOAL_COLOR = 'blue'
config_GOAL_RUN_CM = 9.0
config_TURN_SETTLE_S = 0.15
config_TURN_TIMEOUT_S = 3.0
config_FLAT_CONFIRM_S = 0.4
config_CREST_DROP = 8
config_RAMP_TIMEOUT_S = 6.0
config_STAIRS_TIMEOUT_S = 8.0
config_SERVO_STEP_S = 0.35
config_HOLD_VALID_S = 5.5
config_MATCH_S = 300
config_AUTO_BUDGET_S = 300
config_SEGMENTS = True
config_LEVEL_KEYS = {'up': 1, 'down': 2, 'left': 3, 'right': 4}
config_START_BUTTON = 'b'
config_ABORT_BUTTON = 'a'
config_MENU_BUTTON = 'middle'
config_PAD_DEADZONE = 8
config_PAD_EXPO = 0.5
config_PAD_FAST = 100
config_PAD_SLOW = 45
config_PAD_TURN_GAIN = 0.8
# ---- software/core/geometry.py ----
def geometry_p(key):
    return config_P[key]
def geometry_fwd(key):
    """Distancia hacia adelante desde el CENTRO DE GIRO de un punto medido desde la referencia."""
    return geometry_p(key) - geometry_p('ref_rot_fwd')
def geometry_track_w():
    """Distancia entre los centros de las dos orugas."""
    return geometry_p('robot_w') - geometry_p('belt_w')
def geometry_contact_half():
    """Del centro de giro a cada extremo del tramo apoyado (eje blanco / eje 1a negra)."""
    return geometry_p('track_c') / 2.0
def geometry_nose_fwd():
    """Del centro de giro al eje de la rueda de la nariz (horizontal)."""
    return geometry_contact_half() + geometry_p('track_d') * math.cos(math.radians(geometry_p('nose_deg')))
def geometry_nose_h():
    """Altura del eje de la nariz sobre los ejes apoyados (cm)."""
    return geometry_p('track_d') * math.sin(math.radians(geometry_p('nose_deg')))
def geometry_rear_len():
    """Del centro de giro a la parte trasera (punta trasera de las orugas)."""
    return geometry_contact_half() + geometry_p('wheel_d') / 2.0 + geometry_p('belt_t')
def geometry_track_front():
    """Del centro de giro a la punta delantera de las orugas (la nariz)."""
    return geometry_nose_fwd() + geometry_p('wheel_d') / 2.0 + geometry_p('belt_t')
def geometry_track_len():
    """Largo de la oruga de punta a punta, visto desde arriba."""
    return geometry_rear_len() + geometry_track_front()
def geometry_front_len():
    """Del centro de giro a lo más adelantado del robot (oruga o garra en posición de viaje)."""
    claw = geometry_claw_reach(geometry_p('h_carry')) + geometry_p('claw_tip')
    track = geometry_track_front()
    return claw if claw > track else track
def geometry_robot_len():
    return geometry_rear_len() + geometry_front_len()
def geometry_cm_per_deg():
    """cm que avanza la oruga por grado de encoder."""
    return math.pi * geometry_p('sprocket_d') / 360.0 * geometry_p('slip')
def geometry_cm_per_deg_lr():
    """(izquierda, derecha): cm por grado de cada oruga. No avanzan igual por grado (prueba de
    1 m: 63.5 y 65); su promedio es cm_per_deg()."""
    k = geometry_cm_per_deg()
    r = geometry_p('lr_ratio')
    return (k * 2.0 * r / (1.0 + r), k * 2.0 / (1.0 + r))
geometry_ARM_ANGLES = (0, 45, 90)
def geometry__arm_rows():
    return ((0, geometry_p('arm_h0'), geometry_p('arm_x0')), (45, geometry_p('arm_h45'), geometry_p('arm_x45')), (90, geometry_p('arm_h90'), geometry_p('arm_x90')))
def geometry_arm_at(servo):
    """(altura de la garra sobre el piso, cm delante de la nariz) con el servo a ese ángulo."""
    rows = geometry__arm_rows()
    if servo <= rows[0][0]:
        return (rows[0][1], rows[0][2])
    for i in range(len(rows) - 1):
        (a0, h0, x0) = rows[i]
        (a1, h1, x1) = rows[i + 1]
        if servo <= a1:
            f = (servo - a0) / float(a1 - a0)
            return (h0 + f * (h1 - h0), x0 + f * (x1 - x0))
    return (rows[-1][1], rows[-1][2])
def geometry_arm_servo(h):
    """Ángulo del servo del brazo para la altura h (cm sobre el piso); fuera del alcance, el tope."""
    rows = geometry__arm_rows()
    if h <= rows[0][1]:
        return rows[0][0]
    for i in range(len(rows) - 1):
        (a0, h0, _) = rows[i]
        (a1, h1, _) = rows[i + 1]
        if h <= h1:
            return a0 + (h - h0) / (h1 - h0) * (a1 - a0)
    return rows[-1][0]
def geometry_claw_reach(h):
    """Distancia horizontal del centro de giro al centro de la garra, con la garra a la altura h."""
    return geometry_track_front() + geometry_arm_at(geometry_arm_servo(h))[1]
def geometry_turn_advance():
    """Lo que hay que avanzar tras detectar un cruce con las sondas para girar SOBRE él."""
    return geometry_fwd('sensor_fwd')
def geometry_reach_advance(target, h):
    """Avance para que el punto de agarre (garra a la altura h) llegue a un objeto cuyo centro
    está a `target` cm del centro de giro, en la dirección en la que mira el robot."""
    return target - geometry_claw_reach(h)
def geometry_pedestal_advance(h, from_cross, traveled):
    """Sin ultrasónico: avance para dejar la garra sobre el centro de un pedestal que está a
    `from_cross` cm de un cruce, cuando el robot ya avanzó `traveled` cm desde que las sondas
    detectaron ese cruce (en ese momento el centro de giro estaba sensor_fwd atrás)."""
    return from_cross - geometry_claw_reach(h) + geometry_fwd('sensor_fwd') - traveled
def geometry_tower_advance(traveled):
    """Avance (siguiendo la línea) hasta quedar a side_margin de la torre, medido desde que las
    sondas salieron de la zona verde y ya se avanzaron `traveled` cm."""
    return geometry_p('tower_from_edge') - geometry_front_len() - geometry_p('side_margin') + geometry_fwd('sensor_fwd') - traveled
def geometry_ramp_extra():
    """Avance después de quedar plano arriba de una rampa para estar completamente dentro."""
    return geometry_rear_len() + geometry_p('ramp_margin')
def geometry_zone_extra():
    """Avance desde que las sondas entran a una zona hasta que todo el robot está dentro."""
    return geometry_fwd('sensor_fwd') + geometry_rear_len() + geometry_p('ramp_margin')
def geometry_detour_out():
    """Desplazamiento lateral para rodear la torre sin tocarla."""
    return geometry_p('robot_w') / 2.0 + geometry_p('tower_size') / 2.0 + geometry_p('side_margin')
def geometry_detour_pass():
    """Avance paralelo para rebasar la torre por completo, partiendo de side_margin antes de ella."""
    return geometry_front_len() + geometry_p('tower_size') + geometry_rear_len() + 2 * geometry_p('side_margin')
def geometry_card_to_center(card_odo, now_odo):
    """Cuánto moverse (cm, negativo = atrás) para dejar el centro de giro frente a una tarjeta
    detectada cuando la odometría marcaba card_odo."""
    return card_odo + geometry_fwd('card_probe_fwd') - now_odo
def geometry_probe_x(probe):
    """Posición lateral de cada sonda (cm, negativo = izquierda)."""
    i = config_PROBES.index(probe)
    return geometry_probe_lat(i, geometry_p('probe_pitch'), geometry_p('probe_outer'))
def geometry_probe_lat(i, pitch, outer):
    """Sonda i (0..3, de izquierda a derecha) -> cm laterales (no están igual de separadas)."""
    inner = pitch / 2.0
    return (-inner - outer, -inner, inner, inner + outer)[i]
# ---- software/core/hal.py ----
try:
    import gamepad
    hal__HAS_PAD = True
except ImportError:
    hal__HAS_PAD = False
hal_PROBES = config_PROBES
hal__heading_zero = 0
hal__odo_zero = 0
hal__abort = False
def hal_clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v
def hal_init():
    global hal__abort
    hal__abort = False
    hal_stop()
    hal_reset_heading()
    hal_reset_odometer()
    mbuild.quad_rgb_sensor.set_led(color='white')
    cyberpi.led.on(0, 0, 0, id='all')
def hal_now():
    return cyberpi.timer.get()
hal__on_wait = []
def hal_wait(s):
    for f in hal__on_wait:
        f()
    time.sleep(s)
hal__last_cmd = [0.0, 0.0]
hal__last_t = 0.0
hal__boost = 0.0
hal__last_odo = 0.0
def hal__tip_guard(left, right):
    """Control contra volcadura. Inclinado (rampa, escalera), un arranque brusco empuja el centro
    de masa detrás de la punta de las orugas y el robot se voltea hacia atrás: la potencia solo
    puede SUBIR tip_slew %/s. Bajar la potencia siempre está permitido.

    No se recorta la potencia en pendientes fuertes: en una rampa de 40° eso dejaba al robot sin
    fuerza a media subida (lo mostró el simulador). El otro control es el brazo (gripper.climb_pose)."""
    global hal__last_t
    t = hal_now()
    dt = t - hal__last_t
    hal__last_t = t
    global hal__boost, hal__last_odo
    mean = (left + right) / 2.0
    diff = (left - right) / 2.0
    prev = (hal__last_cmd[0] + hal__last_cmd[1]) / 2.0
    odo = hal_odo_abs()
    v = (odo - hal__last_odo) / dt if dt > 0.001 else 0.0
    hal__last_odo = odo
    if hal_pitch() >= config_P['tip_pitch'] and mean > 0:
        if v < config_P['slope_min_v']:
            hal__boost = hal_clamp(hal__boost + config_P['slope_boost'] * hal_clamp(dt, 0.0, 0.2), 0, 60)
        else:
            hal__boost = hal_clamp(hal__boost - config_P['slope_boost'] * hal_clamp(dt, 0.0, 0.2), 0, 60)
        mean = mean + hal__boost
    else:
        hal__boost = 0.0
    if abs(hal_pitch()) >= config_P['tip_pitch']:
        step = config_P['tip_slew'] * hal_clamp(dt, 0.0, 0.2)
        if abs(mean) > abs(prev) + step:
            mean = prev + step if mean > prev else prev - step
    hal__last_cmd[0] = mean + diff
    hal__last_cmd[1] = mean - diff
    return (mean + diff, mean - diff)
def hal_drive(left, right):
    (left, right) = hal__tip_guard(left, right)
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
    left = hal_clamp(left, -100, 100)
    right = hal_clamp(right, -100, 100)
    mbot2.drive_power(config_LEFT_SIGN * left, config_RIGHT_SIGN * right)
def hal_stop():
    hal__last_cmd[0] = 0.0
    hal__last_cmd[1] = 0.0
    mbot2.drive_power(0, 0)
def hal__encoder_deg():
    """Avance del robot en grados de encoder, CON signo (adelante +, atrás -).

    Es el promedio de las dos orugas con su signo, así que un giro en el lugar no suma
    (antes se usaba abs() y los giros cambiaban la odometría, lo que movía las distancias
    medidas desde un cruce o una tarjeta)."""
    a = config_LEFT_SIGN * mbot2.EM_get_angle('EM1')
    b = config_RIGHT_SIGN * mbot2.EM_get_angle('EM2')
    return (a + b) / 2
def hal_tracks_cm():
    """(izquierda, derecha): avance de cada oruga en cm desde el arranque, con signo."""
    (kl, kr) = geometry_cm_per_deg_lr()
    return (config_LEFT_SIGN * mbot2.EM_get_angle('EM1') * kl, config_RIGHT_SIGN * mbot2.EM_get_angle('EM2') * kr)
def hal_reset_odometer():
    global hal__odo_zero
    hal__odo_zero = hal__encoder_deg()
def hal_odo_abs():
    """Avance absoluto (cm desde el arranque, con signo); no se reinicia."""
    return hal__encoder_deg() * geometry_cm_per_deg()
def hal_odometer_cm():
    """Avance (cm, con signo) desde el último reset_odometer()."""
    return (hal__encoder_deg() - hal__odo_zero) * geometry_cm_per_deg()
def hal_servo(port, angle):
    mbot2.servo_set(hal_clamp(angle, 0, 180), port)
def hal__raw_heading():
    return config_HEADING_SIGN * cyberpi.get_rotation('z') * config_P.get('gyro_scale', 1.0)
def hal_reset_heading():
    global hal__heading_zero
    hal__heading_zero = hal__raw_heading()
def hal_heading():
    return hal__raw_heading() - hal__heading_zero
def hal_pitch():
    return config_PITCH_SIGN * cyberpi.get_pitch()
def hal_probe_light(probe):
    return mbuild.quad_rgb_sensor.get_light(probe)
def hal_darkness():
    """Dict sonda -> 0..1 (0 = blanco, 1 = negro) según la calibración LINE_CAL."""
    out = {}
    for pr in hal_PROBES:
        (white, black) = config_LINE_CAL[pr]
        span = white - black
        if span < 1:
            span = 1
        out[pr] = hal_clamp((white - hal_probe_light(pr)) / span, 0, 1)
    return out
def hal_offset_from(dk):
    """Centro ponderado de la oscuridad -> -100..100 (función pura, probada en PC).

    Se resta el fondo (la sonda más clara): sobre una zona oscura (azul, verde) todas las sondas
    suben igual y la línea se perdía en el promedio (la curva del nivel 3 empieza dentro de la
    zona azul; lo mostró el simulador)."""
    base = 1.0
    for i in range(4):
        if dk[hal_PROBES[i]] < base:
            base = dk[hal_PROBES[i]]
    pi = config_P['probe_pitch'] / 2.0
    po = pi + config_P.get('probe_outer', config_P['probe_pitch'])
    lat = (-po, -pi, pi, po)
    total = 0
    acc = 0
    for i in range(4):
        d = dk[hal_PROBES[i]] - base
        total += d
        acc += d * lat[i]
    if total < 0.05:
        return 0
    return hal_clamp(acc / total / po * 100, -100, 100)
def hal_line_offset():
    if config_P['line_mode'] == 1:
        return hal_offset_from(hal_darkness())
    return config_LINE_SIGN * mbuild.quad_rgb_sensor.get_offset_track(config_LINE_SENSOR)
def hal_line_read():
    """(offset, bits) con UNA sola lectura de las sondas (para el lazo de control)."""
    if config_P['line_mode'] == 1:
        dk = hal_darkness()
        th = config_P['dark_th']
        bits = {}
        for pr in hal_PROBES:
            bits[pr] = dk[pr] >= th
        bits['L2x'] = dk['L2'] >= config_P['cross_th']
        bits['R2x'] = dk['R2'] >= config_P['cross_th']
        return (hal_offset_from(dk), bits)
    return (hal_line_offset(), hal_line_bits())
def hal_line_bits():
    """Dict sonda -> True si ve línea negra."""
    out = {}
    if config_P['line_mode'] == 1:
        dk = hal_darkness()
        th = config_P['dark_th']
        for pr in hal_PROBES:
            out[pr] = dk[pr] >= th
        return out
    sta = mbuild.quad_rgb_sensor.get_line_sta('all', config_LINE_SENSOR)
    for pr in hal_PROBES:
        out[pr] = sta & config_LINE_BIT[pr] != 0
    return out
def hal_probe_rgb(probe):
    q = mbuild.quad_rgb_sensor
    return (q.get_red(probe), q.get_green(probe), q.get_blue(probe))
def hal_probe_color_builtin(probe):
    return mbuild.quad_rgb_sensor.get_color_sta(probe)
def hal_distance_cm():
    """Distancia del Ultrasónico 2, o -1 si el robot no lo lleva (HAS_US = False)."""
    if not config_HAS_US:
        return -1
    return mbuild.ultrasonic2.get(index=config_US_SENSOR)
def hal_button(name):
    return cyberpi.controller.is_press(name)
def hal_abort_requested():
    """Botón de abortar (cfg.ABORT_BUTTON) = detener el autónomo (para pedir reinicio)."""
    global hal__abort
    if not hal__abort and cyberpi.controller.is_press(config_ABORT_BUTTON):
        hal__abort = True
    return hal__abort
def hal_clear_abort():
    global hal__abort
    hal__abort = False
def hal_show(text, line=0, size=16):
    cyberpi.display.show_label(text, size, 0, line * 18, index=line)
def hal_leds(r, g, b):
    cyberpi.led.on(r, g, b, id='all')
def hal_pad_ok():
    return hal__HAS_PAD
def hal_pad_key(name):
    if not hal__HAS_PAD:
        return False
    return gamepad.is_key_pressed(name)
def hal_pad_axis(name):
    if not hal__HAS_PAD:
        return 0
    return gamepad.get_joystick(name)
# ---- software/manual/manual.py ----
manual_claw = config_P['claw_open']
manual_arm_deg = geometry_arm_servo(config_P['h_top'])
manual_fast = True
manual_hold = None
manual_ARM_RATE = 60.0
manual_CLAW_RATE = 150.0
manual_DT = 0.02
def manual_shape(v):
    """Zona muerta + curva expo: más precisión cerca del centro del joystick."""
    if abs(v) < config_PAD_DEADZONE:
        return 0
    x = hal_clamp(v / 100.0, -1, 1)
    y = (1 - config_PAD_EXPO) * x + config_PAD_EXPO * x * x * x
    return y * 100
def manual_mix(forward, turn, limit):
    """Arcade -> tanque, escalando para no saturar."""
    left = forward + turn
    right = forward - turn
    m = max(abs(left), abs(right), 100)
    return (left * limit / m, right * limit / m)
def manual__edge(state, key):
    """True solo en el momento en que se presiona la tecla."""
    now = hal_pad_key(key)
    was = state.get(key, False)
    state[key] = now
    return now and (not was)
def manual__heading_hold(f, t):
    """Corrección de giro para ir recto con el giroscopio mientras no se pide girar."""
    global manual_hold
    if t != 0 or f == 0:
        manual_hold = None
        return t
    if manual_hold is None:
        manual_hold = hal_heading()
        return 0
    err = manual_hold - hal_heading()
    return hal_clamp(config_P['head_kp'] * err, -25, 25)
manual_CLAW_SQUEEZE = 20.0
def manual__claw_dir():
    """+1 si abrir es subir el ángulo del servo (según la calibración claw_open / claw_cube)."""
    return 1 if config_P['claw_open'] >= config_P['claw_cube'] else -1
def manual__claw_limit(deg):
    """La garra va de claw_open (abierta) a claw_cube + CLAW_SQUEEZE (apretando): abrir de más
    o cerrar de más fuerza el servo contra el tope."""
    shut = config_P['claw_cube'] - manual__claw_dir() * manual_CLAW_SQUEEZE
    return hal_clamp(deg, min(config_P['claw_open'], shut), max(config_P['claw_open'], shut))
def manual__arm(deg):
    global manual_arm_deg
    manual_arm_deg = hal_clamp(deg, 0, 90)
def manual_step(state):
    global manual_claw, manual_fast, manual_hold
    if manual__edge(state, 'N1'):
        manual_fast = not manual_fast
        if manual_fast:
            hal_leds(0, 0, 120)
        else:
            hal_leds(120, 60, 0)
    limit = config_PAD_FAST if manual_fast else config_PAD_SLOW
    if hal_pad_key('L1'):
        manual__arm(manual_arm_deg + manual_ARM_RATE * manual_DT)
    elif hal_pad_key('L2'):
        manual__arm(manual_arm_deg - manual_ARM_RATE * manual_DT)
    ry = manual_shape(hal_pad_axis('Ry'))
    if ry:
        manual__arm(manual_arm_deg + ry / 100.0 * manual_ARM_RATE * manual_DT)
    if manual__edge(state, 'N2'):
        manual__arm(geometry_arm_servo(config_P['h_bar']))
    if manual__edge(state, 'N4'):
        manual__arm(geometry_arm_servo(config_P['h_top']))
    if hal_pad_key('R1'):
        manual_claw = manual__claw_limit(manual_claw + manual__claw_dir() * manual_CLAW_RATE * manual_DT)
    elif hal_pad_key('R2'):
        manual_claw = manual__claw_limit(manual_claw - manual__claw_dir() * manual_CLAW_RATE * manual_DT)
    hal_servo(config_SERVO_ARM, manual_arm_deg)
    hal_servo(config_SERVO_CLAW, manual_claw)
    if hal_pad_key('N3'):
        if manual_hold is None:
            manual_hold = hal_heading()
        corr = hal_clamp(config_P['head_kp'] * (manual_hold - hal_heading()), -25, 25)
        v = config_P['v_stairs']
        hal_drive(v + corr, v - corr)
        return
    f = config_PAD_Y_SIGN * manual_shape(hal_pad_axis('Ly'))
    t = config_PAD_TURN_GAIN * manual_shape(hal_pad_axis('Rx'))
    if f == 0 and t == 0:
        slow = config_PAD_SLOW * 0.6
        if hal_pad_key('Up'):
            f = slow
        elif hal_pad_key('Down'):
            f = -slow
        elif hal_pad_key('Left'):
            t = -slow
        elif hal_pad_key('Right'):
            t = slow
        limit = config_PAD_SLOW
    t = manual__heading_hold(f, t)
    (left, right) = manual_mix(f, t, limit)
    hal_drive(left, right)
def manual_status():
    (h, _) = geometry_arm_at(manual_arm_deg)
    return '%s brazo %d (%.0f cm)' % ('RAPIDO' if manual_fast else 'PRECISO', manual_arm_deg, h)
def manual_loop(duration_s=None):
    """Bucle de control manual. Sale al pasar duration_s o con A (abortar) en el CyberPi."""
    global manual_hold
    manual_hold = None
    hal_clear_abort()
    hal_show('MANUAL', line=0, size=24)
    if not hal_pad_ok():
        hal_show('Sin modulo gamepad', line=2)
    state = {}
    t0 = hal_now()
    shown = 0
    while duration_s is None or hal_now() - t0 < duration_s:
        if hal_abort_requested():
            break
        manual_step(state)
        shown += 1
        if shown % 10 == 0:
            hal_show(manual_status(), line=1)
        hal_wait(manual_DT)
    hal_stop()
# ---- software/controls/gripper.py ----
def gripper__move(port, angle):
    hal_servo(port, angle)
    hal_wait(config_SERVO_STEP_S)
def gripper_claw(angle):
    gripper__move(config_SERVO_CLAW, angle)
def gripper_open_claw():
    gripper_claw(config_P['claw_open'])
def gripper_arm_to(h):
    """Garra a la altura h (cm sobre el piso)."""
    gripper__move(config_SERVO_ARM, geometry_arm_servo(h))
def gripper_home():
    """Posición de viaje: brazo a 90° (garra sobre la nariz, lo más compacto para colocar el
    robot en las zonas de inicio, que son más chicas que él), garra abierta."""
    gripper_arm_to(config_P['h_top'])
    gripper_open_claw()
def gripper_hold(seconds=None):
    """Mantiene el objeto quieto el tiempo que pide el reglamento (>= 5 s)."""
    if seconds is None:
        seconds = config_HOLD_VALID_S
    hal_stop()
    t0 = hal_now()
    while hal_now() - t0 < seconds:
        if hal_abort_requested():
            return False
        hal_show('Sosteniendo %d' % int(seconds - (hal_now() - t0) + 1), line=2)
        hal_wait(0.1)
    return True
# ---- software/core/colors.py ----
colors_LED_RGB = {'black': (0, 0, 0), 'white': (255, 255, 255), 'red': (255, 0, 0), 'yellow': (255, 200, 0), 'green': (0, 255, 0), 'blue': (0, 0, 255), 'cyan': (0, 255, 255), 'purple': (180, 0, 255)}
def colors_nearest(rgb, refs):
    """Nombre del color de referencia más cercano (distancia sobre cromaticidad + brillo).

    Normalizar por el brillo hace la clasificación menos sensible a cambios de luz
    (reglamento: el robot debe adaptarse a la iluminación)."""
    (r, g, b) = rgb
    total = r + g + b + 1
    best = None
    best_d = None
    for name in refs:
        (rr, rg, rb) = refs[name]
        rt = rr + rg + rb + 1
        dc = (r / total - rr / rt) ** 2 + (g / total - rg / rt) ** 2 + (b / total - rb / rt) ** 2
        db = ((total - rt) / 765.0) ** 2
        d = dc * 4 + db
        if best_d is None or d < best_d:
            best = name
            best_d = d
    return best
def colors_normalize(name):
    """Nombres del sensor -> nombres del programa (el sensor puede devolver mayúsculas)."""
    if name is None:
        return 'unknown'
    return str(name).strip().lower()
def colors_read(probe):
    if config_P['color_mode'] == 1:
        return colors_nearest(hal_probe_rgb(probe), config_COLOR_REF)
    return colors_normalize(hal_probe_color_builtin(probe))
def colors_read_stable(probe, n=None):
    """Lee hasta tener n lecturas iguales seguidas (máx. 4n intentos)."""
    if n is None:
        n = config_COLOR_CONFIRM
    last = None
    count = 0
    for _ in range(n * 4):
        c = colors_read(probe)
        if c == last:
            count += 1
        else:
            last = c
            count = 1
        if count >= n:
            return c
        hal_wait(config_LOOP_DT)
    return last
def colors_show(name):
    rgb = colors_LED_RGB.get(name, (40, 40, 40))
    hal_leds(rgb[0], rgb[1], rgb[2])
    hal_show(name, line=3, size=24)
def colors_zone_level(color):
    """Nivel de inicio según el color de la zona en la que está el robot (0 = no se sabe)."""
    return config_ZONE_LEVEL.get(color, 0)
def colors_is_card_color(name):
    """Una tarjeta es cualquier color que no sea el piso blanco ni la línea negra."""
    return name not in ('white', 'black', 'unknown', '')
# ---- software/controls/motion.py ----
def motion_sign(v):
    if v < 0:
        return -1
    return 1
def motion_turn(deg, timeout=None):
    """Giro relativo en el lugar; positivo = derecha."""
    if timeout is None:
        timeout = config_TURN_TIMEOUT_S
    target = hal_heading() + deg
    t0 = hal_now()
    settled = None
    while hal_now() - t0 < timeout:
        if hal_abort_requested():
            break
        err = target - hal_heading()
        if abs(err) <= config_P['turn_tol']:
            hal_stop()
            if settled is None:
                settled = hal_now()
            elif hal_now() - settled >= config_TURN_SETTLE_S:
                return True
        else:
            settled = None
            p = config_P['turn_kp'] * err
            p = motion_sign(p) * hal_clamp(abs(p), config_P['v_turn_min'], config_P['v_turn_max'])
            hal_drive(p, -p)
        hal_wait(config_LOOP_DT)
    hal_stop()
    return False
def motion_pivot(deg, speed=None, timeout=None):
    """Giro relativo sobre UNA oruga (la otra detenida); positivo = derecha. Avanza la oruga de
    afuera: la de la derecha queda quieta al girar a la derecha. Si la oruga que se mueve patina
    (recortes del pasillo) el robot solo gira más lento: el centro del giro es la oruga quieta y
    la posición final depende solo del ángulo (en un giro en el lugar una oruga que patina corre
    el robot hasta 6 cm sin que la odometría lo vea). speed < 0 = pivote hacia atrás."""
    if timeout is None:
        timeout = config_TURN_TIMEOUT_S * 2
    if speed is None:
        speed = config_P['v_turn_max']
    target = hal_heading() + deg
    t0 = hal_now()
    settled = None
    while hal_now() - t0 < timeout:
        if hal_abort_requested():
            break
        err = target - hal_heading()
        if abs(err) <= config_P['turn_tol']:
            hal_stop()
            if settled is None:
                settled = hal_now()
            elif hal_now() - settled >= config_TURN_SETTLE_S:
                return True
        else:
            settled = None
            p = hal_clamp(abs(config_P['turn_kp'] * err) * 2, config_P['v_turn_min'] * 1.5, abs(speed))
            fwd = speed > 0
            if err > 0:
                hal_drive(p, 0) if fwd else hal_drive(0, -p)
            else:
                hal_drive(0, p) if fwd else hal_drive(-p, 0)
        hal_wait(config_LOOP_DT)
    hal_stop()
    return False
def motion_wrap180(deg):
    while deg > 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return deg
def motion_turn_to(heading, timeout=None):
    """Gira hasta un rumbo absoluto por el camino más corto (antes podía dar la vuelta larga)."""
    return motion_turn(motion_wrap180(heading - hal_heading()), timeout)
def motion__hold(speed, target):
    corr = config_P['head_kp'] * (target - hal_heading())
    hal_drive(speed + corr, speed - corr)
def motion_straight_until(cond, speed=None, timeout=5.0, target=None):
    """Avanza recto (manteniendo el rumbo) hasta que cond() sea verdadero."""
    if speed is None:
        speed = config_P['v_straight']
    if target is None:
        target = hal_heading()
    t0 = hal_now()
    while hal_now() - t0 < timeout:
        if hal_abort_requested():
            break
        if cond():
            hal_stop()
            return True
        motion__hold(speed, target)
        hal_wait(config_LOOP_DT)
    hal_stop()
    return False
def motion_straight(cm, speed=None, timeout=None, target=None):
    """Avanza (cm > 0) o retrocede (cm < 0) una distancia medida con encoders.

    target: rumbo (hal.heading) que se sostiene con el giroscopio. Sin target se sostiene el
    rumbo con el que empezó; con target, el error del giro anterior y los golpes del piso se
    corrigen durante el avance."""
    if speed is None:
        speed = config_P['v_straight']
    speed = abs(speed) * motion_sign(cm)
    if timeout is None:
        timeout = 1.0 + abs(cm) / 8.0
    hal_reset_odometer()
    goal = abs(cm)
    return motion_straight_until(lambda : abs(hal_odometer_cm()) >= goal, speed, timeout, target)
def motion_climb(speed=None, timeout=None, extra_cm=None):
    """Sube una rampa o escalera: espera a inclinarse, luego a quedar plano, y avanza
    extra_cm para quedar completamente dentro del nivel."""
    if speed is None:
        speed = config_P['v_ramp']
    if timeout is None:
        timeout = config_RAMP_TIMEOUT_S
    if extra_cm is None:
        extra_cm = geometry_ramp_extra()
    target = hal_heading()
    t0 = hal_now()
    climbing = False
    flat_since = None
    while hal_now() - t0 < timeout:
        if hal_abort_requested():
            hal_stop()
            return False
        p = hal_pitch()
        if not climbing:
            if p >= config_P['pitch_ramp']:
                climbing = True
        elif p <= config_P['pitch_flat']:
            if flat_since is None:
                flat_since = hal_now()
            elif hal_now() - flat_since >= config_FLAT_CONFIRM_S:
                hal_stop()
                return motion_straight(extra_cm, config_P['v_straight'])
        else:
            flat_since = None
        motion__hold(speed, target)
        hal_wait(config_LOOP_DT)
    hal_stop()
    return False
def motion_stairs(extra_cm=None):
    return motion_climb(config_P['v_stairs'], config_STAIRS_TIMEOUT_S, extra_cm)
def motion_sees_color(name):
    """Alguna de las cuatro sondas ve ese color. La línea pasa por encima de las zonas, así que
    las centrales leen negro; las exteriores van a los lados (lo mostró el simulador)."""
    for pr in config_PROBES:
        if colors_read(pr) == name:
            return True
    return False
def motion_climb_crest(after_cm, speed=None, timeout=None, target=None, color=None, arm_h=None):
    """Sube hasta pasar la cima (el pitch ya subió y empieza a bajar: el frente está arriba) y
    avanza after_cm medidos con encoder. Sirve cuando arriba no hay espacio para esperar a quedar
    plano (nivel 4: la torre está a ~20 cm de la escalera; lo mostró el simulador)."""
    if speed is None:
        speed = config_P['v_stairs']
    if timeout is None:
        timeout = config_STAIRS_TIMEOUT_S
    if target is None:
        target = hal_heading()
    t0 = hal_now()
    top = 0
    while hal_now() - t0 < timeout:
        if hal_abort_requested():
            hal_stop()
            return False
        p = hal_pitch()
        if p > top:
            top = p
        if color is not None:
            if top >= config_P['pitch_ramp'] and motion_sees_color(color):
                if arm_h is not None:
                    hal_servo(config_SERVO_ARM, geometry_arm_servo(arm_h))
                return motion_straight(after_cm, config_P['v_ramp'], target=target)
        elif top >= config_P['pitch_ramp'] and p <= top - config_CREST_DROP:
            return motion_straight(after_cm, config_P['v_ramp'], target=target)
        motion__hold(speed, target)
        hal_wait(config_LOOP_DT)
    hal_stop()
    return False
def motion_spin_to_line(direction=1, timeout=3.0, leave=True):
    """Gira en el lugar (1 = derecha, -1 = izquierda) hasta que las sondas centrales vean línea.

    leave=True: si ya está sobre una línea, primero sale de ella (para buscar OTRA línea).
    leave=False: si ya ve la línea, termina (después de un giro en un cruce las sondas, que van
    cerca del centro de giro, ya quedan sobre la línea nueva)."""
    t0 = hal_now()
    while leave and hal_now() - t0 < timeout * 0.3:
        b = hal_line_bits()
        if not (b['L1'] or b['R1']):
            break
        hal_drive(direction * config_P['v_turn_min'] * 1.5, -direction * config_P['v_turn_min'] * 1.5)
        hal_wait(config_LOOP_DT)
    while hal_now() - t0 < timeout:
        if hal_abort_requested():
            break
        b = hal_line_bits()
        if b['L1'] or b['R1']:
            hal_stop()
            return True
        hal_drive(direction * config_P['v_turn_min'] * 1.5, -direction * config_P['v_turn_min'] * 1.5)
        hal_wait(config_LOOP_DT)
    hal_stop()
    return False
def motion_on_line():
    b = hal_line_bits()
    return b['L1'] or b['R1']
def motion_seek_line(speed=None, max_cm=60, target=None):
    """Avanza recto (con giroscopio) hasta que las sondas centrales vean una línea."""
    if speed is None:
        speed = config_P['v_slow']
    hal_reset_odometer()
    t = 1.0 + max_cm / 8.0
    found = [False]

    def cond():
        if motion_on_line():
            found[0] = True
            return True
        return abs(hal_odometer_cm()) >= max_cm
    motion_straight_until(cond, speed, t, target)
    return found[0]
def motion_drive_to_edge(color, speed=None, max_cm=30, target=None):
    """Avanza recto (IMU) hasta SALIR de una zona de color: referencia fija para empezar una
    maniobra sin importar dónde quedó o se colocó el robot dentro de la zona."""
    if speed is None:
        speed = config_P['v_slow']
    hal_reset_odometer()
    state = {'seen': False, 'out': 0, 'ok': False}

    def cond():
        inside = motion_sees_color(color)
        if inside:
            state['seen'] = True
            state['out'] = 0
        elif state['seen']:
            state['out'] += 1
            if state['out'] >= config_COLOR_CONFIRM:
                state['ok'] = True
                return True
        return abs(hal_odometer_cm()) >= max_cm
    motion_straight_until(cond, speed, 1.0 + max_cm / 6.0, target)
    return state['ok']
def motion_drive_to_cross(speed=None, max_cm=15, target=None):
    """Avanza recto (IMU) hasta que las DOS sondas exteriores ven negro fuerte: una línea que
    cruza el camino. No es seguir la línea: solo sirve de referencia para corregir la distancia.
    Las marcas verdes y el tope no cuentan (cross_th). Devuelve True si encontró el cruce."""
    if speed is None:
        speed = config_P['v_slow']
    hal_reset_odometer()
    st = {'found': False, 'l': None, 'r': None}

    def cond():
        (_, bits) = hal_line_read()
        odo = hal_odometer_cm()
        if bits.get('L2x', bits['L2']):
            st['l'] = odo
        if bits.get('R2x', bits['R2']):
            st['r'] = odo
        if st['l'] is not None and st['r'] is not None and (abs(st['l'] - st['r']) <= config_P['cross_win']):
            st['found'] = True
            return True
        return abs(odo) >= max_cm
    motion_straight_until(cond, speed, 1.0 + max_cm / 6.0, target)
    return st['found']
def motion_detour(side=None):
    """Rodea un obstáculo fijo (torre, misión 10) y vuelve a buscar la línea.
    side: -1 = por la izquierda, 1 = por la derecha. Distancias desde geometry.py."""
    if side is None:
        side = -1
    start = hal_heading()
    ok = motion_turn(90 * side)
    ok = ok and motion_straight(geometry_detour_out())
    ok = ok and motion_turn_to(start)
    ok = ok and motion_straight(geometry_detour_pass())
    ok = ok and motion_turn(-90 * side)
    if not ok:
        return False
    if not motion_straight_until(motion_on_line, config_P['v_slow'], timeout=4.0):
        return False
    motion_straight(geometry_turn_advance(), config_P['v_slow'])
    return motion_spin_to_line(side, leave=False)
# ---- software/core/settings.py ----
settings_ACTIONS = ['Calibrar linea', 'Ver sensores', 'Probar giro 90', 'Probar recta 50', 'Restaurar valores']
settings_DEFAULTS = {}
def settings_remember_defaults():
    for k in config_P:
        settings_DEFAULTS[k] = config_P[k]
def settings_snapshot():
    cal = {}
    for pr in config_PROBES:
        cal[pr] = [config_LINE_CAL[pr][0], config_LINE_CAL[pr][1]]
    params = {}
    for k in config_P:
        params[k] = config_P[k]
    return {'P': params, 'LINE_CAL': cal}
def settings_apply(data):
    """Aplica un diccionario guardado; ignora claves desconocidas (versiones viejas)."""
    for k in data.get('P', {}):
        if k in config_P:
            config_P[k] = data['P'][k]
    cal = data.get('LINE_CAL', {})
    for pr in cal:
        if pr in config_LINE_CAL:
            config_LINE_CAL[pr] = [cal[pr][0], cal[pr][1]]
def settings_save():
    try:
        f = open(config_PARAMS_FILE, 'w')
        f.write(json.dumps(settings_snapshot()))
        f.close()
        return True
    except Exception:
        return False
def settings_load():
    try:
        f = open(config_PARAMS_FILE)
        data = json.loads(f.read())
        f.close()
        settings_apply(data)
        return True
    except Exception:
        return False
def settings_cal_from_samples(samples):
    """samples: dict sonda -> lista de lecturas de luz. Devuelve dict sonda -> [blanco, negro]
    o None si alguna sonda no vio contraste (no pasó sobre la línea)."""
    out = {}
    for pr in config_PROBES:
        s = samples[pr]
        hi = max(s)
        lo = min(s)
        if hi - lo < 15:
            return None
        out[pr] = [hi, lo]
    return out
def settings_calibrate_line():
    """Con el robot centrado sobre la línea: gira ~45° a cada lado leyendo las 4 sondas."""
    hal_show('Calibrando linea...', line=2)
    samples = {}
    for pr in config_PROBES:
        samples[pr] = []
    start = hal_heading()
    v = config_P['v_turn_min'] * 1.3
    for (target, d) in ((45, 1), (-45, -1), (0, 1)):
        t0 = hal_now()
        while hal_now() - t0 < 4:
            h = hal_heading() - start
            if d > 0 and h >= target or (d < 0 and h <= target):
                break
            hal_drive(d * v, -d * v)
            for pr in config_PROBES:
                samples[pr].append(hal_probe_light(pr))
            hal_wait(config_LOOP_DT)
    hal_stop()
    cal = settings_cal_from_samples(samples)
    if cal is None:
        hal_show('Sin contraste: centrar', line=2)
        return False
    for pr in cal:
        config_LINE_CAL[pr] = cal[pr]
    hal_show('Linea OK', line=2)
    return True
def settings_show_sensors():
    """Muestra oscuridad por sonda, offset y distancia hasta presionar B."""
    hal_wait(0.3)
    while not hal_button('b'):
        dk = hal_darkness()
        hal_show('%.2f %.2f %.2f %.2f' % (dk['L2'], dk['L1'], dk['R1'], dk['R2']), line=1)
        hal_show('offset %d' % hal_offset_from(dk), line=2)
        hal_show('pitch %d  rumbo %d' % (hal_pitch(), hal_heading()), line=3)
        hal_wait(0.05)
    hal_wait(0.3)
def settings_run_action(name):
    if name == 'Calibrar linea':
        return settings_calibrate_line()
    if name == 'Ver sensores':
        settings_show_sensors()
        return True
    if name == 'Probar giro 90':
        return motion_turn(90)
    if name == 'Probar recta 50':
        return motion_straight(50)
    if name == 'Restaurar valores':
        for k in settings_DEFAULTS:
            config_P[k] = settings_DEFAULTS[k]
        return True
    return False
def settings_items():
    out = []
    for t in config_TUNE:
        out.append(('param', t))
    for a in settings_ACTIONS:
        out.append(('action', a))
    return out
def settings_bump(spec, direction):
    """Cambia el parámetro spec = (grupo, clave, paso, min, max) un paso en esa dirección."""
    key = spec[1]
    v = config_P[key] + direction * spec[2]
    v = hal_clamp(v, spec[3], spec[4])
    if isinstance(spec[2], float):
        v = round(v, 3)
    else:
        v = int(v)
    config_P[key] = v
    return v
def settings_try_param(spec):
    """Prueba física de un parámetro: el brazo va a la altura, la garra a la apertura."""
    key = spec[1]
    if key.startswith('h_'):
        gripper_arm_to(config_P[key])
    elif key.startswith('claw_'):
        gripper_claw(config_P[key])
    elif key.startswith('arm_'):
        gripper_arm_to(config_P['h_carry'])
def settings_menu():
    lst = settings_items()
    i = 0
    hal_wait(0.3)
    while True:
        (kind, it) = lst[i]
        if kind == 'param':
            hal_show('%s  %d/%d' % (it[0], i + 1, len(lst)), line=0)
            hal_show('%s = %s' % (it[1], config_P[it[1]]), line=1, size=16)
            hal_show('<- -> valor  A prueba', line=2)
        else:
            hal_show('Accion  %d/%d' % (i + 1, len(lst)), line=0)
            hal_show(it, line=1, size=16)
            hal_show('A = ejecutar', line=2)
        hal_show('B = guardar y salir', line=3)
        if hal_button('down'):
            i = (i + 1) % len(lst)
            hal_wait(0.2)
        elif hal_button('up'):
            i = (i - 1) % len(lst)
            hal_wait(0.2)
        elif kind == 'param' and hal_button('right'):
            settings_bump(it, 1)
            hal_wait(0.15)
        elif kind == 'param' and hal_button('left'):
            settings_bump(it, -1)
            hal_wait(0.15)
        elif hal_button('a'):
            if kind == 'param':
                settings_try_param(it)
            else:
                settings_run_action(it)
            hal_wait(0.3)
        elif hal_button('b'):
            ok = settings_save()
            hal_show('Guardado' if ok else 'No se pudo guardar', line=3)
            hal_wait(0.8)
            hal_clear_abort()
            return
        hal_wait(0.02)
# ---- software/robot/rover_x_manual.py ----
def wait_start():
    """Espera B (y que se suelte, para que no cuente como otra pulsación)."""
    hal_show('MANUAL', line=0, size=24)
    hal_show('%s = empezar' % config_START_BUTTON.upper(), line=1)
    hal_show('%s = parar' % config_ABORT_BUTTON.upper(), line=2)
    hal_show('gamepad: %s' % ('listo' if hal_pad_ok() else 'NO conectado'), line=3)
    while not hal_button(config_START_BUTTON):
        hal_wait(0.02)
    while hal_button(config_START_BUTTON):
        hal_wait(0.02)
def main():
    settings_remember_defaults()
    settings_load()
    hal_init()
    gripper_home()
    while True:
        wait_start()
        manual_loop()
        hal_stop()
        while hal_button(config_ABORT_BUTTON):
            hal_wait(0.02)
        hal_clear_abort()
main()
