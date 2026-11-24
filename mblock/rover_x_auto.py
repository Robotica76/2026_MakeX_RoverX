# rover_x_auto.py bundled for mBlock with tools/scripts/bundle_for_mblock.py (generated:
# edit the source files and bundle again). Modules: config.py, geometry.py, hal.py, colors.py, motion.py, linefollow.py, gripper.py, missions.py, odometry.py, autonomous.py, settings.py
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
# ---- software/controls/linefollow.py ----
linefollow__last_err = 0
linefollow__integral = 0
linefollow__pending_turn = 0
linefollow__win_start = None
linefollow__seen_l = False
linefollow__seen_r = False
linefollow__clear_since = None
linefollow__armed = True
linefollow_last_junction = None
linefollow_last_edge = None
linefollow__after_turn = False
linefollow_bias_cm = 0.0
def linefollow_reset():
    global linefollow__last_err, linefollow__integral, linefollow__pending_turn
    linefollow__last_err = 0
    linefollow__integral = 0
    linefollow__pending_turn = 0
    linefollow_reset_junctions()
def linefollow_reset_junctions():
    global linefollow__win_start, linefollow__seen_l, linefollow__seen_r, linefollow__clear_since, linefollow__armed
    linefollow__win_start = None
    linefollow__seen_l = False
    linefollow__seen_r = False
    linefollow__clear_since = None
    linefollow__armed = True
def linefollow_correction(err, last_err, integral):
    """Corrección PID (función pura para poder probarla en PC)."""
    return config_P['kp'] * err + config_P['ki'] * integral + config_P['kd'] * (err - last_err)
def linefollow_drive_with(err, speed):
    global linefollow__last_err, linefollow__integral
    if linefollow_bias_cm:
        po = config_P['probe_pitch'] / 2.0 + config_P.get('probe_outer', config_P['probe_pitch'])
        err -= linefollow_bias_cm / po * 100.0
    linefollow__integral = hal_clamp(linefollow__integral + err, -200, 200)
    c = linefollow_correction(err, linefollow__last_err, linefollow__integral)
    linefollow__last_err = err
    hal_drive(speed + c, speed - c)
def linefollow_step(speed):
    """Un ciclo de control. Devuelve los bits leídos."""
    (err, bits) = hal_line_read()
    linefollow_drive_with(err, speed)
    return bits
def linefollow_sees_line(bits):
    return bits['L2'] or bits['L1'] or bits['R1'] or bits['R2']
def linefollow_track_junction(bits, odo):
    """Actualiza el rastreador con una lectura. Devuelve "X", "L", "R" al cerrar un cruce, o None."""
    global linefollow__win_start, linefollow__seen_l, linefollow__seen_r, linefollow__clear_since, linefollow__armed, linefollow_last_junction
    left = bits.get('L2x', bits['L2'])
    right = bits.get('R2x', bits['R2'])
    outer = left or right
    if not linefollow__armed:
        if outer:
            linefollow__clear_since = None
        elif linefollow__clear_since is None:
            linefollow__clear_since = odo
        elif odo - linefollow__clear_since >= 1.5:
            linefollow__armed = True
        return None
    if outer:
        if linefollow__win_start is None:
            linefollow__win_start = odo
        linefollow__seen_l = linefollow__seen_l or left
        linefollow__seen_r = linefollow__seen_r or right
    if linefollow__win_start is None:
        return None
    if linefollow__seen_l and linefollow__seen_r or odo - linefollow__win_start >= config_P['cross_win']:
        kind = 'X' if linefollow__seen_l and linefollow__seen_r else 'L' if linefollow__seen_l else 'R'
        linefollow_last_junction = (kind, linefollow__win_start)
        linefollow__win_start = None
        linefollow__seen_l = False
        linefollow__seen_r = False
        linefollow__armed = False
        linefollow__clear_since = None
        return kind
    return None
def linefollow__check_green():
    global linefollow__pending_turn
    left = colors_read('L2') == 'green'
    right = colors_read('R2') == 'green'
    if left and right:
        linefollow__pending_turn = 2
    elif left:
        linefollow__pending_turn = -1
    elif right:
        linefollow__pending_turn = 1
def linefollow_turn_at_junction(deg):
    """Avanza hasta que el CENTRO DE GIRO quede sobre el último cruce, gira y se alinea."""
    done = 0
    if linefollow_last_junction is not None:
        done = hal_odo_abs() - linefollow_last_junction[1]
    adv = geometry_turn_advance() - done
    if abs(adv) > 0.5:
        motion_straight(adv, config_P['v_slow'])
    if deg == 0:
        return True
    motion_turn(deg * 0.85)
    ok = motion_spin_to_line(1 if deg > 0 else -1, timeout=2.0, leave=False)
    linefollow_reset()
    global linefollow__after_turn
    linefollow__after_turn = True
    return ok
def linefollow_follow_until(cond, speed=None, timeout=10.0, green=False):
    """Sigue la línea hasta que cond(bits, junction) sea verdadero.

    junction es "X"/"L"/"R" en el ciclo en que se cierra un cruce, si no None.
    Si la línea se pierde más de P["lost_s"] segundos, devuelve False."""
    global linefollow__pending_turn, linefollow__after_turn, linefollow__armed
    if speed is None:
        speed = config_P['v_line']
    linefollow_reset()
    if linefollow__after_turn:
        linefollow__after_turn = False
        linefollow__armed = False
    t0 = hal_now()
    lost_since = None
    while hal_now() - t0 < timeout:
        if hal_abort_requested():
            break
        (err, bits) = hal_line_read()
        j = linefollow_track_junction(bits, hal_odo_abs())
        if cond(bits, j):
            hal_stop()
            return True
        if green and config_P['green'] == 1:
            if linefollow__pending_turn == 0:
                linefollow__check_green()
            elif j is not None:
                t = linefollow__pending_turn
                linefollow__pending_turn = 0
                linefollow_turn_at_junction(180 if t == 2 else 90 * t)
                continue
        if linefollow_sees_line(bits):
            lost_since = None
        elif lost_since is None:
            lost_since = hal_now()
        elif hal_now() - lost_since > config_P['lost_s']:
            break
        linefollow_drive_with(err, speed)
        hal_wait(config_LOOP_DT)
    hal_stop()
    return False
def linefollow_at_junction(count=1, kind='any'):
    """Cuenta cruces; kind = "any", "X", "L" o "R" ("L" también cuenta las "X")."""
    state = {'n': 0}

    def cond(bits, j):
        if j is not None and (kind == 'any' or j == kind or j == 'X'):
            state['n'] += 1
        return state['n'] >= count
    return cond
def linefollow_at_color(name, probes=('L1', 'R1')):

    def cond(bits, j):
        for pr in probes:
            if colors_read(pr) == name:
                return True
        return False
    return cond
def linefollow_color_then_cm(name, cm, probes=None):
    """Verdadero `cm` después de que alguna sonda vio ese color. Sirve para terminar arriba de una
    rampa SIN detenerse sobre la pendiente (parado ahí, el robot se resbalaba hacia atrás al
    arrancar; lo mostró el simulador con una rampa de 40°)."""
    if probes is None:
        probes = config_PROBES
    state = {'from': None}

    def cond(bits, j):
        if state['from'] is None:
            for pr in probes:
                if colors_read(pr) == name:
                    state['from'] = hal_odo_abs()
                    break
            return False
        return hal_odo_abs() - state['from'] >= cm
    return cond
def linefollow_after_cm(cm):
    start = hal_odo_abs()

    def cond(bits, j):
        return hal_odo_abs() - start >= cm
    return cond
def linefollow_leave_color(name, probes=None):
    """Verdadero cuando las sondas, después de haber visto la zona de ese color, la dejan de ver
    (3 lecturas seguidas).
    Guarda la odometría en last_edge: es la referencia para medir desde el borde de la zona."""
    if probes is None:
        probes = config_PROBES
    state = {'n': 0, 'seen': False}

    def cond(bits, j):
        global linefollow_last_edge
        inside = False
        for pr in probes:
            if colors_read(pr) == name:
                inside = True
        if inside:
            state['seen'] = True
        if not state['seen']:
            return False
        state['n'] = 0 if inside else state['n'] + 1
        if state['n'] >= config_COLOR_CONFIRM:
            linefollow_last_edge = hal_odo_abs()
            return True
        return False
    return cond
def linefollow_obstacle_within(cm):
    """Solo con ultrasónico (HAS_US = True)."""

    def cond(bits, j):
        d = hal_distance_cm()
        return 0 < d <= cm
    return cond
def linefollow_line_end(cm=2.0):
    """Verdadero cuando ninguna sonda ve línea durante `cm` de avance (en una curva cerrada la
    línea se pierde un instante y no es el final; lo mostró el simulador)."""
    state = {'since': None}

    def cond(bits, j):
        if linefollow_sees_line(bits):
            state['since'] = None
            return False
        odo = hal_odo_abs()
        if state['since'] is None:
            state['since'] = odo
        return odo - state['since'] >= cm
    return cond
def linefollow_pitch_above(deg):

    def cond(bits, j):
        return hal_pitch() >= deg
    return cond
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
# ---- software/autonomous/missions.py ----
missions_L1_SOUTH_CM = 33.8
missions_L1_EAST_CM = 59.0
missions_L1_CROSS_SLACK = 6.0
missions_L1_NORTH_CM = 18.4
missions_L1_TO_RAMP_CM = 18.7
missions_L1_START = (12.6, 12.9)
missions_L1_TURN_X = 15.4
missions_L1_CROSS_CY = 43.2
missions_L1_TURN_BACK_CM = 1.5
missions_L1_CUBE_Y = 47.4
missions_L1_PIVOT_BACK_CM = 11.0
missions_L1_PIVOT_D = 11.9
missions_L1_BACK_CM = 4.5
missions_L2_TO_PED_CM = 8.8
missions_L2_T_BACK_CM = 3.0
missions_L2_BACK_CM = 6.0
missions_T2_PLACED = (131.9, 30.4)
missions_T2_FROM_M4 = (133.9, 26.8)
missions_CORRIDOR_TURN = (134.2, 43.5)
missions_CORRIDOR_ENTRY_Y = 45.0
missions_CORRIDOR_DIAG_HEAD = -26.0
missions_CORRIDOR_DIAG_PT = (212.7, 16.0)
missions_CORRIDOR_DIAG_START_X = 150.5
missions_CORRIDOR_DIAG_MAX_CM = 110.0
missions_CORRIDOR_DIAG_V = 14
missions_CORRIDOR_DIAG_KP = 2.5
missions_CORRIDOR_DIAG_DET = (208.96, 17.82)
missions_CORRIDOR_EXIT_Y = 15.9
missions_CORRIDOR_DIAG_X = 218.6
missions_CORRIDOR_TURN_X = 223.0
missions_RAMP2_X = 228.15
missions_RAMP2_LINE_X = 228.6
missions_RAMP2_BACK_CM = 8.0
missions_CORRIDOR_RAMP_CM = 5.5
missions_CORRIDOR_TURN_HEAD = 60
missions_CORRIDOR = [('goto', missions_CORRIDOR_TURN[0], missions_CORRIDOR_TURN[1]), ('head', 0), ('track', 0, 'y', missions_CORRIDOR_ENTRY_Y, missions_CORRIDOR_DIAG_START_X - missions_CORRIDOR_TURN[0]), ('pivot', missions_CORRIDOR_DIAG_HEAD), ('diag', missions_CORRIDOR_DIAG_PT[0], missions_CORRIDOR_DIAG_PT[1], missions_CORRIDOR_DIAG_HEAD, missions_CORRIDOR_DIAG_MAX_CM, missions_CORRIDOR_DIAG_DET[0], missions_CORRIDOR_DIAG_DET[1], missions_CORRIDOR_DIAG_V, missions_CORRIDOR_DIAG_KP), ('pivot', 0), ('track_line', 0, 'y', missions_CORRIDOR_EXIT_Y, 60), ('fix', 'x', missions_CORRIDOR_DIAG_X), ('track', 0, 'y', missions_CORRIDOR_EXIT_Y, missions_CORRIDOR_TURN_X - missions_CORRIDOR_DIAG_X), ('head', missions_CORRIDOR_TURN_HEAD), ('drive', missions_CORRIDOR_TURN_HEAD, missions_CORRIDOR_RAMP_CM, 'v_slow'), ('head', 90), ('fix_line', 'x', missions_RAMP2_LINE_X, 1), ('straight', -missions_RAMP2_BACK_CM, 'v_slow')]
missions_L3_IN_CM = 19.2
missions_L3_PRISM_SOUTH = 3.6
missions_L3_PRISM_BACK = 4.0
missions_L3_PRISM_GRAB = 6.5
missions_L3_PRISM_HEAD = 107
missions_L3_PRISM_CARRY = 22.0
missions_L3_PRISM_LEAVE = 8.0
missions_L3_EDGE = (228.8, 70.8)
missions_L3_STAIRS_POINT = (166.1, 96.4)
missions_PRISM_MANEUVER = [('edge', 'blue', 'v_slow', 30, 90), ('pose', missions_L3_EDGE[0], missions_L3_EDGE[1], 90), ('drive', 90, missions_L3_PRISM_SOUTH), ('arm', 'h_top'), ('head', 180), ('straight', -missions_L3_PRISM_BACK, 'v_slow'), ('claw', 'claw_open'), ('arm', 'h_prism'), ('drive', 180, missions_L3_PRISM_GRAB), ('head', missions_L3_PRISM_HEAD), ('drive', missions_L3_PRISM_HEAD, missions_L3_PRISM_CARRY), ('arm', 'h_carry'), ('straight', -missions_L3_PRISM_LEAVE, 'v_slow')]
missions_STAIRS_BACK_CM = 5.0
missions_TO_STAIRS = [('head', -90), ('straight', -missions_STAIRS_BACK_CM, 'v_slow'), ('cross', 'v_slow', 15, -90), ('center',), ('head', 180), ('line_ref', 180, ('pitch',), 'v_slow', 10)]
missions_L3_RETRY_TO_STAIRS = [('ref', 90), ('edge', 'blue', 'v_slow', 30, 90), ('pose', missions_L3_EDGE[0], missions_L3_EDGE[1], 90), ('goto', missions_L3_STAIRS_POINT[0], missions_L3_STAIRS_POINT[1])]
missions_BALL_HEAD = 113
missions_LINE_HEAD_AT_MARK2 = -152
missions_BALL_STEPS = [('line', ('color', 'red'), 'v_slow', 20), ('center',), ('head', missions_BALL_HEAD), ('arm', 'h_pedestal'), ('claw', 'claw_open'), ('approach_obj', 'h_pedestal', 'ball_ped_dist'), ('claw', 'claw_ball'), ('arm', 'h_carry'), ('retreat',), ('hold',), ('head', missions_LINE_HEAD_AT_MARK2)]
missions_STAIRS_TOP_CM = 8.0
missions_TOWER_WEST_CM = 37.0
missions_CARDS_BACK_CM = 7.0
missions_TOWER_DETOUR = [('drive', 110, 20), ('drive', 180, missions_TOWER_WEST_CM), ('seek', 'v_slow', 25, -90), ('center',), ('head', 180)]
def missions_junctions(route):
    """Tabla (cruces, tipo, giro) -> pasos. Ya no la usa el nivel 1; queda para rutas por línea."""
    steps = []
    for (n, kind, deg) in route:
        steps.append(('line', ('junction', n, kind), 'v_line', 10))
        steps.append(('jturn', deg))
    return steps
missions_MISSIONS = [{'id': 1, 'level': 1, 'pts': 50, 'name': 'Seguir linea', 'critical': True, 'steps': [('arm', 'h_top'), ('claw', 'claw_open'), ('ref', 90), ('pose', missions_L1_START[0], missions_L1_START[1], 90), ('track', 90, 'x', missions_L1_TURN_X, missions_L1_SOUTH_CM - missions_L1_CROSS_SLACK, 'v_line'), ('cross', 'v_slow', 2 * missions_L1_CROSS_SLACK, 90), ('fix', 'y', missions_L1_CROSS_CY), ('straight', -missions_L1_TURN_BACK_CM, 'v_slow'), ('head', 0), ('track', 0, 'y', missions_L1_CUBE_Y, missions_L1_EAST_CM - missions_L1_CROSS_SLACK - (missions_L1_TURN_X - missions_L1_START[0]), 'v_line'), ('cross', 'v_slow', 2 * missions_L1_CROSS_SLACK, 0)]}, {'id': 2, 'level': 1, 'pts': 50, 'name': 'Tomar cubo', 'critical': False, 'steps': [('approach_ped', 'h_pedestal', 'ped1_from_cross'), ('arm', 'h_pedestal'), ('claw', 'claw_cube'), ('arm', 'h_carry'), ('retreat',), ('hold',)]}, {'id': 3, 'level': 1, 'pts': 50, 'name': 'Subir nivel 2', 'critical': True, 'steps': [('straight', -missions_L1_PIVOT_BACK_CM, 'v_slow'), ('pivot', -90), ('drive', -90, missions_L1_NORTH_CM - missions_L1_PIVOT_D - (49.0 - missions_L1_CUBE_Y)), ('drive', 0, missions_L1_TO_RAMP_CM - missions_L1_PIVOT_D + (missions_L1_PIVOT_BACK_CM - missions_L1_BACK_CM)), ('line', ('pitch',), 'v_slow', 6), ('arm', 'h_climb'), ('line', ('junction', 1, 'any'), 'v_ramp', 8), ('center',), ('arm', 'h_carry')]}, {'id': 4, 'level': 1, 'pts': 100, 'name': 'Dejar cubo', 'critical': False, 'end_level': True, 'steps': [('ref', 0), ('jturn', -90), ('line', ('cm', missions_L2_TO_PED_CM), 'v_slow', 8), ('arm', 'h_pedestal'), ('claw', 'claw_open'), ('arm', 'h_carry'), ('straight', -missions_L2_BACK_CM, 'v_slow'), ('head', 90), ('pose', missions_T2_FROM_M4[0], missions_T2_FROM_M4[1], None)]}, {'id': 5, 'level': 2, 'pts': 100, 'name': 'Zona obstaculos', 'critical': True, 'steps': [('if_placed', [('ref', 0), ('straight', -missions_L2_T_BACK_CM, 'v_slow'), ('jturn', 90), ('pose', missions_T2_PLACED[0], missions_T2_PLACED[1], None)])] + missions_CORRIDOR}, {'id': 6, 'level': 2, 'pts': 100, 'name': 'Subir nivel 3', 'critical': True, 'end_level': True, 'steps': [('arm', 'h_climb'), ('ramp_up', 90, missions_RAMP2_X, 'blue', missions_L3_IN_CM, 'v_ramp', 60)]}, {'id': 7, 'level': 3, 'pts': 100, 'name': 'Mover prisma', 'critical': False, 'steps': [('ref', 90, 3)] + missions_PRISM_MANEUVER}, {'id': 9, 'level': 3, 'pts': 150, 'name': 'Escalera nivel 4', 'critical': True, 'steps': [('if_skipped', 7, missions_L3_RETRY_TO_STAIRS), ('if_ran', 7, [('goto', missions_L3_STAIRS_POINT[0], missions_L3_STAIRS_POINT[1])])] + missions_TO_STAIRS + [('arm', 'h_climb'), ('crest', missions_STAIRS_TOP_CM, 180, 'green', 'h_top')]}, {'id': 10, 'level': 4, 'pts': 100, 'name': 'Evitar torre', 'critical': True, 'steps': [('ref', 180, 4), ('arm', 'h_top')] + missions_TOWER_DETOUR}, {'id': 11, 'level': 4, 'pts': 120, 'name': 'Colores', 'critical': False, 'steps': [('cards', missions_CARDS_BACK_CM)]}, {'id': 13, 'level': 4, 'pts': 100, 'name': 'Meta azul', 'critical': False, 'steps': [('stop_in', 'blue')]}]
def missions_total_points():
    t = 0
    for m in missions_MISSIONS:
        t += m['pts']
    return t
# ---- software/controls/odometry.py ----
odometry__x = 0.0
odometry__y = 0.0
odometry__off = 0.0
odometry__last = None
odometry_slip_cm = 0.0
def odometry_set_pose(x, y, heading):
    """El robot está en (x, y) cm mirando a `heading` grados de mapa."""
    global odometry__x, odometry__y, odometry__off, odometry__last, odometry_slip_cm
    odometry__x = x
    odometry__y = y
    odometry__off = heading - hal_heading()
    (l, r) = hal_tracks_cm()
    odometry__last = (l, r, hal_heading())
    odometry_slip_cm = 0.0
def odometry_heading():
    return hal_heading() + odometry__off
def odometry_pose():
    return (odometry__x, odometry__y, odometry_heading())
def odometry_update():
    """Integra el movimiento desde la última llamada (llamarla en cada ciclo de control)."""
    global odometry__x, odometry__y, odometry__last, odometry_slip_cm
    (l, r) = hal_tracks_cm()
    h = hal_heading()
    if odometry__last is None:
        odometry__last = (l, r, h)
        return
    a = l - odometry__last[0]
    b = r - odometry__last[1]
    dh = h - odometry__last[2]
    odometry__last = (l, r, h)
    dpsi = -dh * math.pi / 180.0
    B = geometry_track_w() * config_P['skid']
    resid = dpsi - (b - a) / B
    if abs(b) < 0.15 * abs(a) and abs(a) > 1e-06:
        ds = b - dpsi * B / 2.0
    elif abs(a) < 0.15 * abs(b):
        ds = a + dpsi * B / 2.0
    elif abs(b - a) > abs(a + b) or abs(resid) < config_P['slip_tol'] * math.pi / 180.0:
        ds = (a + b) / 2.0
    elif resid * (a + b) > 0:
        ds = b - dpsi * B / 2.0
    else:
        ds = a + dpsi * B / 2.0
    odometry_slip_cm += (a + b) / 2.0 - ds
    hm = (h - dh / 2.0 + odometry__off) * math.pi / 180.0
    odometry__x += ds * math.cos(hm)
    odometry__y += ds * math.sin(hm)
hal__on_wait.append(odometry_update)
def odometry_wrap(d):
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d
def odometry_goto(x, y, speed=None, tol=1.0, timeout=None):
    """Va al punto (x, y) del mapa: gira hacia él y avanza corrigiendo el rumbo hacia el punto
    con la posición estimada (si un recorte lo desvía, el controlador lo regresa). Termina al
    llegar o al rebasarlo. Devuelve True si llegó."""
    if speed is None:
        speed = config_P['v_slow']
    odometry_update()
    dx = x - odometry__x
    dy = y - odometry__y
    d0 = math.sqrt(dx * dx + dy * dy)
    if d0 < tol:
        return True
    if timeout is None:
        timeout = 2.0 + d0 / 5.0
    bearing = math.atan2(dy, dx) * 180.0 / math.pi
    if abs(odometry_wrap(bearing - odometry_heading())) > 10:
        motion_turn_to(hal_heading() + odometry_wrap(bearing - odometry_heading()))
        odometry_update()
    ux = dx / d0
    uy = dy / d0
    t0 = hal_now()
    while hal_now() - t0 < timeout:
        if hal_abort_requested():
            break
        odometry_update()
        dx = x - odometry__x
        dy = y - odometry__y
        d = math.sqrt(dx * dx + dy * dy)
        if d < tol or dx * ux + dy * uy <= 0:
            hal_stop()
            return True
        err = odometry_wrap(math.atan2(dy, dx) * 180.0 / math.pi - odometry_heading())
        corr = config_P['head_kp'] * err
        sp = speed
        if d < 6:
            sp = max(config_P['v_turn_min'], speed * d / 6.0)
        hal_drive(sp + corr, sp - corr)
        hal_wait(config_LOOP_DT)
    hal_stop()
    return False
def odometry_track(x0, y0, hdg, speed=None, timeout=5.0, max_cm=None, stop=None, max_deg=None, halt=True):
    """Sigue la recta del mapa que pasa por (x0, y0) con rumbo `hdg`, corrigiendo el rumbo Y el
    desvío lateral (si el patinaje lo empuja de lado, regresa a la recta). Termina cuando stop()
    es verdadero, al recorrer max_cm o al acabarse el tiempo. Devuelve True si fue por stop().
    max_deg: ángulo máximo contra la recta al corregir el desvío (por omisión track_max).
    halt=False: al cumplirse stop() no frena (para encadenar otro tramo sin parar en una rampa).
    speed < 0: hacia atrás por la misma recta (rumbo hdg = hacia donde mira el robot)."""
    if speed is None:
        speed = config_P['v_slow']
    odometry_update()
    rad = hdg * math.pi / 180.0
    ux = math.cos(rad)
    uy = math.sin(rad)
    (sx, sy) = (odometry__x, odometry__y)
    t0 = hal_now()
    while hal_now() - t0 < timeout:
        if hal_abort_requested():
            break
        odometry_update()
        if stop is not None and stop():
            if halt:
                hal_stop()
            return True
        along = (odometry__x - sx) * ux + (odometry__y - sy) * uy
        if speed < 0:
            along = -along
        if max_cm is not None and along >= max_cm:
            break
        cross = ux * (odometry__y - y0) - uy * (odometry__x - x0)
        lim = config_P.get('track_max', 30) if max_deg is None else max_deg
        k = -1 if speed < 0 else 1
        want = hdg - k * hal_clamp(config_P['track_k'] * cross, -lim, lim)
        err = odometry_wrap(want - odometry_heading())
        corr = config_P['head_kp'] * err
        hal_drive(speed + corr, speed - corr)
        hal_wait(config_LOOP_DT)
    hal_stop()
    return False
# ---- software/autonomous/autonomous.py ----
autonomous_cards = []
autonomous_results = []
autonomous_last_advance = 0
autonomous_heading_ref = 0
autonomous_goal_entry = None
autonomous_run_level = 1
autonomous_current_level = 1
autonomous_pass_skipped = set()
autonomous_pass_ran = set()
autonomous_placed = True
autonomous_next_level = 1
autonomous_completed = set()
def autonomous_reset_state():
    """Todo lo que una corrida anterior pudo dejar (al relanzar el autónomo desde el menú sin
    apagar el robot, una referencia vieja movía el primer giro; lo mostró el simulador)."""
    global linefollow__after_turn, linefollow_last_edge, linefollow_last_junction
    global autonomous_cards, autonomous_results, autonomous_last_advance, autonomous_heading_ref, autonomous_goal_entry
    autonomous_goal_entry = None
    autonomous_cards = []
    autonomous_results = []
    autonomous_last_advance = 0
    autonomous_heading_ref = 0
    linefollow_reset()
    linefollow_last_junction = None
    linefollow_last_edge = None
    linefollow__after_turn = False
def autonomous_val(x):
    """Número tal cual, o el valor actual del parámetro cfg.P con ese nombre."""
    if isinstance(x, str):
        return config_P[x]
    return x
def autonomous_make_cond(spec):
    kind = spec[0]
    if kind == 'junction':
        return linefollow_at_junction(spec[1], spec[2])
    if kind == 'color':
        return linefollow_at_color(spec[1], config_PROBES)
    if kind == 'color_cm':
        return linefollow_color_then_cm(spec[1], autonomous_val(spec[2]))
    if kind == 'cm':
        return linefollow_after_cm(autonomous_val(spec[1]))
    if kind == 'leave':
        return linefollow_leave_color(spec[1])
    if kind == 'obstacle':
        return linefollow_obstacle_within(autonomous_val(spec[1]))
    if kind == 'end':
        return linefollow_line_end()
    if kind == 'pitch':
        return linefollow_pitch_above(config_P['pitch_ramp'])
    if kind == 'head':
        (target, tol) = (spec[1], spec[2])
        return lambda bits, j: abs(motion_wrap180(target - autonomous_heading_ref - hal_heading())) < tol
    raise ValueError('condicion desconocida: %s' % kind)
def autonomous_red_index(found):
    """Posición (0..5) de la tarjeta roja, o -1."""
    for i in range(len(found)):
        if found[i][0] == 'red':
            return i
    return -1
def autonomous__note_card(c, odo):
    """Registra la tarjeta si no es una ya vista (mismo color a menos de 6 cm: la tarjeta mide
    4.7 cm y al retroceder y volver a pasar se lee en sus dos orillas)."""
    for (c0, o0) in autonomous_cards:
        if c0 == c and abs(o0 - odo) < 6:
            return False
    autonomous_cards.append((c, odo))
    colors_show(c)
    return True
def autonomous__scan_back(back_cm):
    """Retrocede recto (rumbo con el giroscopio) leyendo tarjetas: al volver a la línea
    después de rodear la torre, la sonda ya pasó la primera tarjeta."""
    start = hal_odo_abs()
    h0 = hal_heading()
    t0 = hal_now()
    (last, same) = ('white', 0)
    v = config_P['v_slow']
    while start - hal_odo_abs() < back_cm and hal_now() - t0 < 4:
        if hal_abort_requested():
            break
        c = colors_read(config_CARD_PROBE)
        if c == last:
            same += 1
        else:
            (last, same) = (c, 1)
        if same == config_COLOR_CONFIRM and colors_is_card_color(c):
            autonomous__note_card(c, hal_odo_abs())
        corr = hal_clamp(config_P['head_kp'] * (h0 - hal_heading()), -15, 15)
        hal_drive(-v + corr, -v - corr)
        hal_wait(config_LOOP_DT)
    hal_stop()
def autonomous_scan_cards(back_cm=0):
    """Sigue la línea despacio y registra cada tarjeta de color (color + odometría).
    back_cm: antes, retrocede eso leyendo (tarjetas que quedaron detrás de la sonda)."""
    global autonomous_cards
    autonomous_cards = []
    if back_cm > 0:
        autonomous__scan_back(back_cm)
    linefollow_reset()
    start = hal_odo_abs()
    t0 = hal_now()
    last = 'white'
    same = 0
    goal_from = None
    while hal_now() - t0 < 15 and hal_odo_abs() - start < config_CARD_SCAN_MAX_CM:
        if motion_sees_color(config_GOAL_COLOR):
            if goal_from is None:
                goal_from = hal_odo_abs()
            elif hal_odo_abs() - goal_from >= config_GOAL_RUN_CM:
                global autonomous_goal_entry
                autonomous_goal_entry = goal_from
                break
        else:
            goal_from = None
        if hal_abort_requested():
            break
        c = colors_read(config_CARD_PROBE)
        if c == last:
            same += 1
        else:
            last = c
            same = 1
        odo = hal_odo_abs()
        if same == config_COLOR_CONFIRM and colors_is_card_color(c):
            autonomous__note_card(c, odo)
        linefollow_step(config_P['v_slow'])
        hal_wait(config_LOOP_DT)
    hal_stop()
    if autonomous_goal_entry is not None and autonomous_goal_entry >= start:
        autonomous_cards = [k for k in autonomous_cards if k[1] < autonomous_goal_entry - 1]
    autonomous_cards.sort(key=lambda k: k[1])
    return len(autonomous_cards) > 0
def autonomous_drop_red():
    i = autonomous_red_index(autonomous_cards)
    if i < 0:
        return False
    side = config_P['bar_side']
    ok = motion_straight(geometry_card_to_center(autonomous_cards[i][1], hal_odo_abs()), config_P['v_slow'])
    ok = ok and motion_turn(90 * side)
    gripper_arm_to(config_P['h_bar'])
    ok = ok and motion_straight(config_P['bar_reach'], config_P['v_slow'])
    gripper_open_claw()
    motion_straight(-config_P['bar_reach'], config_P['v_slow'])
    gripper_arm_to(config_P['h_carry'])
    motion_turn(-90 * side)
    motion_spin_to_line(-side, timeout=1.5)
    return ok
def autonomous_stop_in(color):
    """Para completamente dentro de la zona: avanza zone_extra() contado desde que las sondas
    ENTRARON a la zona (si la lectura de tarjetas ya entró, no se vuelve a contar desde aquí;
    antes el robot se pasaba y salía por el otro lado: lo mostró el simulador)."""
    global autonomous_goal_entry
    if autonomous_goal_entry is None:
        start = {'odo': None}

        def cond(bits, j):
            if motion_sees_color(color):
                start['odo'] = hal_odo_abs()
                return True
            return False
        if not linefollow_follow_until(cond, config_P['v_line'], 15):
            return False
        autonomous_goal_entry = start['odo']
    rest = geometry_zone_extra() - (hal_odo_abs() - autonomous_goal_entry)
    if rest > 0.5:
        motion_straight(rest, config_P['v_slow'])
    hal_stop()
    return True
def autonomous__advance(cm):
    global autonomous_last_advance
    autonomous_last_advance = cm
    return motion_straight(cm, config_P['v_slow'])
def autonomous_approach_pedestal(h, from_cross):
    """Garra sobre el centro de un pedestal que está a from_cross cm del último cruce."""
    traveled = 0
    if linefollow_last_junction is not None:
        traveled = hal_odo_abs() - linefollow_last_junction[1]
    return autonomous__advance(geometry_pedestal_advance(autonomous_val(h), autonomous_val(from_cross), traveled))
def autonomous_approach_object(h, dist):
    """Garra sobre un objeto cuyo centro está a dist cm del centro de giro, al frente."""
    return autonomous__advance(geometry_reach_advance(autonomous_val(dist), autonomous_val(h)))
def autonomous_to_tower():
    """Sigue la línea hasta quedar a side_margin de la torre (referencia: borde de la zona verde)."""
    traveled = 0
    if linefollow_last_edge is not None:
        traveled = hal_odo_abs() - linefollow_last_edge
    adv = geometry_tower_advance(traveled)
    if adv < 0:
        return motion_straight(adv, config_P['v_slow'])
    if adv < 0.5:
        return True
    return linefollow_follow_until(linefollow_after_cm(adv), config_P['v_slow'], 8)
autonomous_RAMP_FLAT_DEG = 6.0
autonomous_RAMP_MAX_DEG = 1.5
def autonomous_map_target(deg):
    """Rumbo de mapa -> valor de hal.heading() al que hay que llegar por el camino corto.
    hal.heading() acumula vueltas; sin normalizar, una recta podía "sostener" un rumbo a 360°
    del real y torcerse (lo mostró el simulador)."""
    now = hal_heading()
    return now + motion_wrap180(deg - autonomous_heading_ref - now)
def autonomous_run_step(step):
    global linefollow_bias_cm
    global autonomous_heading_ref
    kind = step[0]
    if kind == 'line':
        green = len(step) > 4 and step[4]
        return linefollow_follow_until(autonomous_make_cond(step[1]), autonomous_val(step[2]), step[3], green)
    if kind == 'jturn':
        return linefollow_turn_at_junction(step[1])
    if kind == 'straight':
        speed = autonomous_val(step[2]) if len(step) > 2 else config_P['v_straight']
        return motion_straight(autonomous_val(step[1]), speed)
    if kind == 'center':
        return motion_straight(geometry_turn_advance(), config_P['v_slow'])
    if kind == 'turn':
        return motion_turn(step[1])
    if kind == 'ref':
        if len(step) > 2 and step[2] != autonomous_run_level:
            return True
        autonomous_heading_ref = step[1] - hal_heading()
        return True
    if kind == 'head':
        return motion_turn_to(autonomous_map_target(step[1]))
    if kind == 'drive':
        target = autonomous_map_target(step[1])
        speed = autonomous_val(step[3]) if len(step) > 3 else config_P['v_slow']
        motion_turn_to(target)
        return motion_straight(autonomous_val(step[2]), speed, target=target)
    if kind == 'pose':
        hdg = step[3] if step[3] is not None else autonomous_heading_ref + hal_heading()
        (ox, oy, _) = odometry_pose()
        odometry_set_pose(ox if step[1] is None else step[1], oy if step[2] is None else step[2], hdg)
        autonomous_heading_ref = hdg - hal_heading()
        return True
    if kind == 'square':
        speed = autonomous_val(step[5]) if len(step) > 5 and step[5] is not None else config_P['v_slow']
        motion_turn_to(autonomous_map_target(step[1]))
        (x, y, _) = odometry_pose()
        if len(step) > 6:
            if step[3] == 'x':
                y = step[6]
            else:
                x = step[6]
        odometry_track(x, y, step[1], speed, timeout=autonomous_val(step[2]))
        hal_stop()
        hal_wait(0.3)
        (x, y, _) = odometry_pose()
        if step[3] == 'x':
            x = step[4]
        else:
            y = step[4]
        odometry_set_pose(x, y, step[1])
        autonomous_heading_ref = step[1] - hal_heading()
        return True
    if kind == 'track_line':
        speed = autonomous_val(step[5]) if len(step) > 5 else config_P['v_slow']
        motion_turn_to(autonomous_map_target(step[1]))
        (x, y, _) = odometry_pose()
        if step[2] == 'x':
            x = step[3]
        else:
            y = step[3]
        return odometry_track(x, y, step[1], speed, timeout=2.0 + autonomous_val(step[4]) / 5.0, max_cm=autonomous_val(step[4]), stop=motion_on_line, max_deg=step[6] if len(step) > 6 else None)
    if kind == 'diag':
        speed = autonomous_val(step[7]) if len(step) > 7 else config_P['v_slow']

        def seen():
            b = hal_line_bits()
            return any((b[pr] for pr in config_PROBES))
        kp0 = config_P['head_kp']
        if len(step) > 8:
            config_P['head_kp'] = autonomous_val(step[8])
        try:
            ok = odometry_track(step[1], step[2], step[3], speed, timeout=2.0 + autonomous_val(step[4]) / 4.0, max_cm=autonomous_val(step[4]), stop=seen)
        finally:
            config_P['head_kp'] = kp0
        if ok:
            (_, _, h) = odometry_pose()
            odometry_set_pose(step[5], step[6], h)
        return ok
    if kind == 'track':
        speed = autonomous_val(step[5]) if len(step) > 5 else config_P['v_slow']
        motion_turn_to(autonomous_map_target(step[1]))
        (x, y, _) = odometry_pose()
        if step[2] == 'x':
            x = step[3]
        else:
            y = step[3]
        odometry_track(x, y, step[1], speed, timeout=2.0 + autonomous_val(step[4]) / 4.0, max_cm=autonomous_val(step[4]))
        return True
    if kind == 'back_to':
        speed = autonomous_val(step[3]) if len(step) > 3 and step[3] is not None else config_P['v_slow']
        i = 0 if step[1] == 'x' else 1
        start = odometry_pose()[i]
        goal = step[2]
        up = goal > start

        def there():
            v = odometry_pose()[i]
            return v >= goal if up else v <= goal
        tmo = 2.0 + abs(goal - start) / 4.0
        if len(step) > 5:
            (x, y, _) = odometry_pose()
            if step[1] == 'x':
                y = step[5]
            else:
                x = step[5]
            return odometry_track(x, y, step[4], -abs(speed), timeout=tmo, stop=there)
        return motion_straight_until(there, -abs(speed), timeout=tmo)
    if kind == 'ramp_up':
        (hdg, x0, color, extra, speed, max_cm) = step[1:7]
        speed = autonomous_val(speed)
        (_, y, _) = odometry_pose()
        odometry_track(x0, y, hdg, speed, timeout=4.0, max_cm=12, max_deg=autonomous_RAMP_FLAT_DEG, halt=False, stop=lambda : abs(hal_pitch()) > 4)
        (_, y, _) = odometry_pose()
        seen = odometry_track(x0, y, hdg, speed, timeout=4.0 + autonomous_val(max_cm) / 3.0, max_cm=autonomous_val(max_cm), stop=lambda : motion_sees_color(color), halt=False, max_deg=autonomous_RAMP_MAX_DEG)
        if not seen:
            hal_stop()
            return False
        (_, y, _) = odometry_pose()
        odometry_track(x0, y, hdg, speed, timeout=2.0 + autonomous_val(extra) / 3.0, max_cm=autonomous_val(extra), max_deg=autonomous_RAMP_MAX_DEG)
        return True
    if kind == 'fix_line':
        po = config_P['probe_pitch'] / 2.0 + config_P.get('probe_outer', config_P['probe_pitch'])
        acc = 0.0
        n = 0
        for _ in range(5):
            (err, bits) = hal_line_read()
            if bits['L1'] or bits['R1']:
                acc += err
                n += 1
            hal_wait(config_LOOP_DT)
        if n < 3:
            return True
        err_cm = acc / n * po / 100.0
        (x, y, h) = odometry_pose()
        v = step[2] + step[3] * err_cm
        if step[1] == 'x':
            x = v
        else:
            y = v
        odometry_set_pose(x, y, h)
        return True
    if kind == 'bias':
        linefollow_bias_cm = step[1]
        return True
    if kind == 'fix':
        (x, y, h) = odometry_pose()
        if step[1] == 'x':
            x = step[2]
        else:
            y = step[2]
        odometry_set_pose(x, y, h)
        return True
    if kind == 'pivot':
        speed = config_P['v_turn_max']
        if 'back' in step[2:]:
            speed = -speed
        return motion_pivot(motion_wrap180(autonomous_map_target(step[1]) - hal_heading()), speed)
    if kind == 'goto':
        speed = autonomous_val(step[3]) if len(step) > 3 else config_P['v_slow']
        return odometry_goto(step[1], step[2], speed)
    if kind == 'edge':
        target = autonomous_map_target(step[4]) if len(step) > 4 else None
        return motion_drive_to_edge(step[1], autonomous_val(step[2]), autonomous_val(step[3]), target)
    if kind == 'cross':
        target = autonomous_map_target(step[3]) if len(step) > 3 else None
        if target is not None:
            motion_turn_to(target)
        return motion_drive_to_cross(autonomous_val(step[1]), autonomous_val(step[2]), target)
    if kind == 'seek':
        speed = autonomous_val(step[1]) if len(step) > 1 else config_P['v_slow']
        max_cm = autonomous_val(step[2]) if len(step) > 2 else 60
        target = None
        if len(step) > 3:
            target = autonomous_map_target(step[3])
            motion_turn_to(target)
        return motion_seek_line(speed, max_cm, target)
    if kind == 'spin':
        return motion_spin_to_line(step[1])
    if kind == 'approach_ped':
        return autonomous_approach_pedestal(step[1], step[2])
    if kind == 'approach_obj':
        return autonomous_approach_object(step[1], step[2])
    if kind == 'retreat':
        return motion_straight(-autonomous_last_advance, config_P['v_slow'])
    if kind == 'to_tower':
        return autonomous_to_tower()
    if kind == 'climb':
        return motion_climb()
    if kind == 'line_ref':
        cond = autonomous_make_cond(step[2])
        acc = [0.0, 0]

        def cond2(bits, j):
            acc[0] += hal_heading()
            acc[1] += 1
            return cond(bits, j)
        ok = linefollow_follow_until(cond2, autonomous_val(step[3]), step[4])
        if acc[1] > 0:
            autonomous_heading_ref = step[1] - acc[0] / acc[1]
        return ok
    if kind == 'if_placed':
        if not autonomous_placed:
            return True
        for sub in step[1]:
            if not autonomous_run_step(sub):
                return False
        return True
    if kind in ('if_skipped', 'if_ran'):
        hit = step[1] in (autonomous_pass_skipped if kind == 'if_skipped' else autonomous_pass_ran)
        if not hit:
            return True
        for sub in step[2]:
            if not autonomous_run_step(sub):
                return False
        return True
    if kind == 'if_start':
        if step[1] != autonomous_run_level:
            return True
        for sub in step[2]:
            if not autonomous_run_step(sub):
                return False
        return True
    if kind == 'crest':
        target = None
        if len(step) > 2:
            target = autonomous_map_target(step[2])
            motion_turn_to(target)
        color = step[3] if len(step) > 3 else None
        arm_h = autonomous_val(step[4]) if len(step) > 4 else None
        return motion_climb_crest(autonomous_val(step[1]), target=target, color=color, arm_h=arm_h)
    if kind == 'stairs':
        return motion_stairs(autonomous_val(step[1]) if len(step) > 1 else None)
    if kind == 'claw':
        gripper_claw(autonomous_val(step[1]))
        return True
    if kind == 'arm':
        gripper_arm_to(autonomous_val(step[1]))
        return True
    if kind == 'hold':
        return gripper_hold()
    if kind == 'detour':
        return motion_detour(step[1])
    if kind == 'cards':
        return autonomous_scan_cards(autonomous_val(step[1]) if len(step) > 1 else 0)
    if kind == 'drop_red':
        return autonomous_drop_red()
    if kind == 'stop_in':
        return autonomous_stop_in(step[1])
    if kind == 'wait':
        hal_wait(step[1])
        return True
    raise ValueError('paso desconocido: %s' % kind)
def autonomous_run_mission(m):
    hal_show('M%d %s' % (m['id'], m['name']), line=1)
    for step in m['steps']:
        if hal_abort_requested():
            return False
        if not autonomous_run_step(step):
            return False
    return True
def autonomous_run(start_level=1, budget_s=None, t_start=None, skip=None):
    """Corre el TRAMO que empieza en start_level. Devuelve los puntos estimados.

    Con cfg.SEGMENTS el robot se detiene al terminar el tramo (misión con "end_level") o al
    fallar una misión crítica; next_level queda con el nivel que conviene elegir después.
    skip: misiones ya logradas en este encendido; solo se saltan las de manipulación (las de
    navegación, críticas, se repiten porque llevan al robot a donde empieza la siguiente)."""
    global autonomous_run_level, autonomous_placed, autonomous_next_level, autonomous_current_level
    if budget_s is None:
        budget_s = config_AUTO_BUDGET_S
    if t_start is None:
        t_start = hal_now()
    autonomous_reset_state()
    autonomous_run_level = start_level
    autonomous_next_level = start_level
    autonomous_placed = True
    crit = {}
    for m in missions_MISSIONS:
        crit[m['id']] = m['critical']
    done = set()
    for mid in skip or ():
        if not crit.get(mid, True):
            done.add(mid)
    points = 0
    autonomous_pass_skipped.clear()
    autonomous_pass_ran.clear()
    for m in missions_MISSIONS:
        if m['level'] < start_level:
            continue
        if m['id'] in done:
            autonomous_pass_skipped.add(m['id'])
            continue
        if hal_now() - t_start >= budget_s or hal_abort_requested():
            break
        autonomous_pass_ran.add(m['id'])
        autonomous_current_level = m['level']
        ok = autonomous_run_mission(m)
        autonomous_placed = False
        autonomous_results.append((m['id'], ok))
        hal_stop()
        if ok:
            done.add(m['id'])
            autonomous_completed.add(m['id'])
            points += m['pts']
            hal_leds(0, 80, 0)
            hal_show('Pts aprox: %d' % points, line=2)
            if m.get('end_level') and config_SEGMENTS:
                autonomous_next_level = m['level'] + 1
                break
            if m is missions_MISSIONS[-1]:
                autonomous_next_level = 5
        else:
            hal_leds(80, 0, 0)
            if hal_abort_requested() or m['critical']:
                break
    hal_stop()
    return points
def autonomous_detect_level():
    """Lee el color de la zona de inicio bajo el robot (rojo/amarillo/azul/verde)."""
    for probe in ('L1', 'R1'):
        lvl = colors_zone_level(colors_read_stable(probe))
        if lvl:
            return lvl
    return 1
def autonomous_choose_level(level=None):
    """Pantalla de inicio del autónomo. Joystick: elige el nivel (cfg.LEVEL_KEYS: arriba 1,
    abajo 2, izquierda 3, derecha 4). START_BUTTON (B): arrancar el tramo. Centro del joystick:
    menú de ajustes. Devuelve el nivel, o 0 = ajustes."""
    if level is None:
        level = autonomous_detect_level()
    hal_wait(0.3)
    while True:
        hal_show('Nivel %d' % level, line=0)
        hal_show('^1 v2 <3 >4  %s=GO' % config_START_BUTTON.upper(), line=1)
        hal_show('%s=parar  centro=ajustes' % config_ABORT_BUTTON.upper(), line=2)
        for key in config_LEVEL_KEYS:
            if hal_button(key):
                level = config_LEVEL_KEYS[key]
        if hal_button(config_START_BUTTON):
            while hal_button(config_START_BUTTON):
                hal_wait(0.02)
            return level
        if hal_button(config_MENU_BUTTON):
            return 0
        hal_wait(0.02)
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
# ---- software/robot/rover_x_auto.py ----
def main():
    settings_remember_defaults()
    loaded = settings_load()
    hal_init()
    hal_show('Parametros: %s' % ('guardados' if loaded else 'de fabrica'), line=3)
    gripper_home()
    level = None
    while True:
        level = autonomous_choose_level(level)
        if level == 0:
            settings_menu()
            level = None
            continue
        hal_clear_abort()
        hal_show('AUTONOMO nivel %d' % level, line=0)
        points = autonomous_run(level, skip=autonomous_completed)
        hal_stop()
        if hal_abort_requested():
            hal_show('PARADO con %s' % config_ABORT_BUTTON.upper(), line=3)
        else:
            hal_show('Fin tramo: ~%d pts' % points, line=3)
        level = autonomous_next_level if autonomous_next_level <= 4 else None
        hal_clear_abort()
        hal_wait(0.4)
main()
