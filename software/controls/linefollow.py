# Rover X 2026: seguidor de línea PID con detección de cruces robusta para línea delgada.
#
# Cruces "con memoria": con una línea de 10-30 mm y el robot oscilando, las dos sondas
# exteriores casi nunca ven negro AL MISMO TIEMPO. Por eso un cruce se arma con lo que vean
# L2 y R2 dentro de una ventana de avance (P["cross_win"] cm, medida con encoders):
#   "X" = las dos exteriores vieron negro, "L" = solo la izquierda, "R" = solo la derecha.
# Los giros se escriben en la ruta (missions.py); las marcas verdes son opcionales (P["green"]).

import hal
import config as cfg
import colors
import motion
import geometry as geo

_last_err = 0
_integral = 0
_pending_turn = 0        # marcas verdes: -1 izq, 1 der, 2 media vuelta

# rastreador de cruces
_win_start = None        # odometría absoluta donde una exterior vio negro por primera vez
_seen_l = False
_seen_r = False
_clear_since = None      # odometría desde que ambas exteriores están en blanco (refractario)
_armed = True
last_junction = None     # (tipo, odometría absoluta del inicio del cruce)
last_edge = None         # odometría absoluta al salir de una zona de color (leave_color)


def reset():
    global _last_err, _integral, _pending_turn
    _last_err = 0
    _integral = 0
    _pending_turn = 0
    reset_junctions()


def reset_junctions():
    global _win_start, _seen_l, _seen_r, _clear_since, _armed
    _win_start = None
    _seen_l = False
    _seen_r = False
    _clear_since = None
    _armed = True


def correction(err, last_err, integral):
    """Corrección PID (función pura para poder probarla en PC)."""
    return cfg.P["kp"] * err + cfg.P["ki"] * integral + cfg.P["kd"] * (err - last_err)


def drive_with(err, speed):
    global _last_err, _integral
    _integral = hal.clamp(_integral + err, -200, 200)
    c = correction(err, _last_err, _integral)
    _last_err = err
    # línea a la izquierda (err < 0) -> frenar la oruga izquierda
    hal.drive(speed + c, speed - c)


def step(speed):
    """Un ciclo de control. Devuelve los bits leídos."""
    err, bits = hal.line_read()
    drive_with(err, speed)
    return bits


def sees_line(bits):
    return bits["L2"] or bits["L1"] or bits["R1"] or bits["R2"]


def track_junction(bits, odo):
    """Actualiza el rastreador con una lectura. Devuelve "X", "L", "R" al cerrar un cruce, o None."""
    global _win_start, _seen_l, _seen_r, _clear_since, _armed, last_junction
    outer = bits["L2"] or bits["R2"]
    if not _armed:
        # esperar a salir del cruce anterior: 1.5 cm con las exteriores en blanco
        if outer:
            _clear_since = None
        elif _clear_since is None:
            _clear_since = odo
        elif odo - _clear_since >= 1.5:
            _armed = True
        return None
    if outer:
        if _win_start is None:
            _win_start = odo
        _seen_l = _seen_l or bits["L2"]
        _seen_r = _seen_r or bits["R2"]
    if _win_start is None:
        return None
    if (_seen_l and _seen_r) or odo - _win_start >= cfg.P["cross_win"]:
        kind = "X" if (_seen_l and _seen_r) else ("L" if _seen_l else "R")
        last_junction = (kind, _win_start)
        _win_start = None
        _seen_l = False
        _seen_r = False
        _armed = False
        _clear_since = None
        return kind
    return None


def _check_green():
    global _pending_turn
    left = colors.read("L2") == "green"
    right = colors.read("R2") == "green"
    if left and right:
        _pending_turn = 2
    elif left:
        _pending_turn = -1
    elif right:
        _pending_turn = 1


def turn_at_junction(deg):
    """Avanza hasta que el CENTRO DE GIRO quede sobre el último cruce, gira y se alinea."""
    done = 0
    if last_junction is not None:
        done = hal.odo_abs() - last_junction[1]
    adv = geo.turn_advance() - done
    if abs(adv) > 0.5:
        motion.straight(adv, cfg.P["v_slow"])
    if deg == 0:
        return True
    motion.turn(deg * 0.85)                    # el último tramo lo termina spin_to_line
    ok = motion.spin_to_line(1 if deg > 0 else -1, timeout=2.0)
    reset()
    return ok


def follow_until(cond, speed=None, timeout=10.0, green=False):
    """Sigue la línea hasta que cond(bits, junction) sea verdadero.

    junction es "X"/"L"/"R" en el ciclo en que se cierra un cruce, si no None.
    Si la línea se pierde más de P["lost_s"] segundos, devuelve False."""
    global _pending_turn
    if speed is None:
        speed = cfg.P["v_line"]
    reset()
    t0 = hal.now()
    lost_since = None
    while hal.now() - t0 < timeout:
        if hal.abort_requested():
            break
        err, bits = hal.line_read()
        j = track_junction(bits, hal.odo_abs())
        if cond(bits, j):
            hal.stop()
            return True
        if green and cfg.P["green"] == 1:
            if _pending_turn == 0:
                _check_green()
            elif j is not None:
                t = _pending_turn
                _pending_turn = 0
                turn_at_junction(180 if t == 2 else 90 * t)
                continue
        if sees_line(bits):
            lost_since = None
        elif lost_since is None:
            lost_since = hal.now()
        elif hal.now() - lost_since > cfg.P["lost_s"]:
            break
        drive_with(err, speed)
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    return False


# ---------------------------------------------------------------- condiciones de parada
def at_junction(count=1, kind="any"):
    """Cuenta cruces; kind = "any", "X", "L" o "R" ("L" también cuenta las "X")."""
    state = {"n": 0}

    def cond(bits, j):
        if j is not None and (kind == "any" or j == kind or j == "X"):
            state["n"] += 1
        return state["n"] >= count
    return cond


def at_color(name, probes=("L1", "R1")):
    def cond(bits, j):
        for pr in probes:
            if colors.read(pr) == name:
                return True
        return False
    return cond


def after_cm(cm):
    start = hal.odo_abs()

    def cond(bits, j):
        return hal.odo_abs() - start >= cm
    return cond


def leave_color(name, probes=("L1", "R1")):
    """Verdadero cuando las sondas dejan de ver la zona de ese color (3 lecturas seguidas).
    Guarda la odometría en last_edge: es la referencia para medir desde el borde de la zona."""
    state = {"n": 0}

    def cond(bits, j):
        global last_edge
        inside = False
        for pr in probes:
            if colors.read(pr) == name:
                inside = True
        state["n"] = 0 if inside else state["n"] + 1
        if state["n"] >= cfg.COLOR_CONFIRM:
            last_edge = hal.odo_abs()
            return True
        return False
    return cond


def obstacle_within(cm):
    """Solo con ultrasónico (HAS_US = True)."""
    def cond(bits, j):
        d = hal.distance_cm()
        return 0 < d <= cm
    return cond


def line_end():
    def cond(bits, j):
        return not sees_line(bits)
    return cond


def pitch_above(deg):
    def cond(bits, j):
        return hal.pitch() >= deg
    return cond
