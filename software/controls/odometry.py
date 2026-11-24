## @file odometry.py
#  @brief Odometría de posición (x, y, rumbo) con encoders + giroscopio, robusta al patinaje.
#  @details Marco del mapa (el de las vistas del reglamento): x hacia el este desde la orilla
#  izquierda, y hacia el SUR desde la orilla de arriba, en cm; rumbo 0 = este, 90 = sur (horario
#  positivo, igual que hal.heading()).
#  Patinaje (recortes de cartón en la zona de obstáculos): la oruga que pisa uno gira sin avanzar
#  y su encoder sigue contando. El giroscopio dice cuánto giró DE VERDAD el robot y los encoders
#  cuánto "debió" girar; la diferencia dice qué oruga patinó, y el avance se mide con la otra:
#    - (adelante) giró más a la izquierda de lo esperado -> patinó la izquierda -> se mide con la derecha
#    - (adelante) giró más a la derecha de lo esperado -> patinó la derecha -> se mide con la izquierda
#    - en reversa los signos se invierten (la que patina avanza menos en su sentido)
#  B = distancia entre orugas * P["skid"] (las orugas derrapan al girar; CALIBRAR).
#  update() se registra en hal._on_wait: se integra en cada ciclo de control.

import math
import hal
import config as cfg
import geometry as geo
import motion

_x = 0.0
_y = 0.0
_off = 0.0               # rumbo de mapa = hal.heading() + _off
_last = None             # (izquierda cm, derecha cm, hal.heading()) de la última actualización
slip_cm = 0.0            # avance que los encoders contaron de más por patinaje (diagnóstico)


## @brief Fija la posición conocida del robot (paso "pose" de las misiones).
#  @param x cm hacia el este.
#  @param y cm hacia el sur.
#  @param heading Rumbo de mapa en grados (0 este, 90 sur).
def set_pose(x, y, heading):
    global _x, _y, _off, _last, slip_cm
    _x = x
    _y = y
    _off = heading - hal.heading()
    l, r = hal.tracks_cm()
    _last = (l, r, hal.heading())
    slip_cm = 0.0


## @brief Rumbo de mapa actual.
#  @return Grados (0 este, 90 sur, 180 oeste, -90 norte).
def heading():
    return hal.heading() + _off


## @brief Posición estimada.
#  @return (x, y, rumbo) en cm y grados de mapa.
def pose():
    return _x, _y, heading()


## @brief Integra el movimiento desde la última llamada.
#  @details Elige con qué oruga medir el avance según el patinaje: en un pivote, la oruga quieta
#  (no puede patinar); en un giro en el lugar o sin patinaje, el promedio; si no, la que no
#  patinó (ver la explicación del archivo). Acumula en slip_cm lo que contaron de más.
def update():
    global _x, _y, _last, slip_cm
    l, r = hal.tracks_cm()
    h = hal.heading()
    if _last is None:
        _last = (l, r, h)
        return
    a = l - _last[0]
    b = r - _last[1]
    dh = h - _last[2]
    _last = (l, r, h)
    dpsi = -dh * math.pi / 180.0                 # giro antihorario real (rad)
    B = geo.track_w() * cfg.P["skid"]
    resid = dpsi - (b - a) / B
    if abs(b) < 0.15 * abs(a) and abs(a) > 1e-6:
        # pivote sobre la oruga derecha (quieta: no puede patinar): el centro gira alrededor
        # de ella aunque la izquierda patine
        ds = b - dpsi * B / 2.0
    elif abs(a) < 0.15 * abs(b):
        ds = a + dpsi * B / 2.0                  # pivote sobre la izquierda
    elif abs(b - a) > abs(a + b) or abs(resid) < cfg.P["slip_tol"] * math.pi / 180.0:
        # giro en el lugar (o sin patinaje): el promedio. Al girar, el derrape de las orugas
        # nunca coincide exacto con el giroscopio y la fórmula de patinaje inventaba avance.
        ds = (a + b) / 2.0
    elif resid * (a + b) > 0:                    # patinó la izquierda (vale adelante y atrás:
        ds = b - dpsi * B / 2.0                  # la que patina avanza menos en su sentido)
    else:                                        # patinó la derecha
        ds = a + dpsi * B / 2.0
    slip_cm += (a + b) / 2.0 - ds
    hm = (h - dh / 2.0 + _off) * math.pi / 180.0
    _x += ds * math.cos(hm)
    _y += ds * math.sin(hm)


# integrar en cada ciclo de control (hal.wait): así cada movimiento se integra con su propio
# rumbo (antes un retroceso y el giro siguiente se integraban juntos y la posición se corría)
hal._on_wait.append(update)


## @brief Lleva un ángulo a -180..180.
#  @param d Grados.
#  @return Grados equivalentes en -180..180.
def wrap(d):
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


## @brief Va a un punto del mapa.
#  @details Gira hacia él y avanza corrigiendo el rumbo hacia el punto con la posición estimada
#  (si un recorte lo desvía, el controlador lo regresa). Termina al llegar o al rebasarlo; frena
#  en los últimos 6 cm.
#  @param x cm hacia el este.
#  @param y cm hacia el sur.
#  @param speed % de potencia (por defecto v_slow).
#  @param tol cm de tolerancia para dar por llegado.
#  @param timeout s máximos (por defecto 2 + distancia / 5).
#  @return True si llegó.
def goto(x, y, speed=None, tol=1.0, timeout=None):
    if speed is None:
        speed = cfg.P["v_slow"]
    update()
    dx = x - _x
    dy = y - _y
    d0 = math.sqrt(dx * dx + dy * dy)
    if d0 < tol:
        return True
    if timeout is None:
        timeout = 2.0 + d0 / 5.0
    bearing = math.atan2(dy, dx) * 180.0 / math.pi
    if abs(wrap(bearing - heading())) > 10:
        motion.turn_to(hal.heading() + wrap(bearing - heading()))
        update()
    ux = dx / d0
    uy = dy / d0
    t0 = hal.now()
    while hal.now() - t0 < timeout:
        if hal.abort_requested():
            break
        update()
        dx = x - _x
        dy = y - _y
        d = math.sqrt(dx * dx + dy * dy)
        if d < tol or dx * ux + dy * uy <= 0:     # llegó o lo rebasó
            hal.stop()
            return True
        err = wrap(math.atan2(dy, dx) * 180.0 / math.pi - heading())
        corr = cfg.P["head_kp"] * err
        sp = speed
        if d < 6:
            sp = max(cfg.P["v_turn_min"], speed * d / 6.0)
        hal.drive(sp + corr, sp - corr)
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    return False


## @brief Sigue una recta del mapa corrigiendo rumbo y desvío lateral.
#  @details Si el patinaje lo empuja de lado, regresa a la recta (ángulo máximo de corrección
#  max_deg). Termina cuando stop() es verdadero, al recorrer max_cm o al acabarse el tiempo.
#  @param x0 x de un punto de la recta (cm).
#  @param y0 y de un punto de la recta (cm).
#  @param hdg Rumbo de la recta (grados de mapa) = hacia donde mira el robot.
#  @param speed % de potencia; negativo = hacia atrás por la misma recta.
#  @param timeout s máximos.
#  @param max_cm cm máximos a recorrer sobre la recta.
#  @param stop Función sin argumentos que dice cuándo terminar (p. ej. "ve la línea").
#  @param max_deg Ángulo máximo contra la recta al corregir (por defecto track_max).
#  @param halt False = al cumplirse stop() no frena (encadenar otro tramo sin parar en una rampa).
#  @return True si terminó por stop().
def track(x0, y0, hdg, speed=None, timeout=5.0, max_cm=None, stop=None, max_deg=None, halt=True):
    if speed is None:
        speed = cfg.P["v_slow"]
    update()
    rad = hdg * math.pi / 180.0
    ux = math.cos(rad)
    uy = math.sin(rad)
    sx, sy = _x, _y
    t0 = hal.now()
    while hal.now() - t0 < timeout:
        if hal.abort_requested():
            break
        update()
        if stop is not None and stop():
            if halt:
                hal.stop()
            return True
        along = (_x - sx) * ux + (_y - sy) * uy
        if speed < 0:
            along = -along
        if max_cm is not None and along >= max_cm:
            break
        cross = ux * (_y - y0) - uy * (_x - x0)       # + = a la derecha de la recta
        lim = cfg.P.get("track_max", 30) if max_deg is None else max_deg
        # hacia atrás la corrección se invierte: para volver a la recta la cola apunta hacia ella
        k = -1 if speed < 0 else 1
        want = hdg - k * hal.clamp(cfg.P["track_k"] * cross, -lim, lim)
        err = wrap(want - heading())
        corr = cfg.P["head_kp"] * err
        hal.drive(speed + corr, speed - corr)
        hal.wait(cfg.LOOP_DT)
    hal.stop()
    return False
