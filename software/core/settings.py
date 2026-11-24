## @file settings.py
#  @brief Menú de ajustes en el CyberPi, calibración de línea y guardado de parámetros.
#  @details Se entra desde la pantalla de inicio del autónomo con el CENTRO del joystick
#  (cfg.MENU_BUTTON). Dentro del menú:
#    - arriba/abajo = elegir parámetro o acción
#    - izquierda/derecha = bajar / subir el valor
#    - A = probar (mueve el brazo o la garra a ese valor, o ejecuta la acción)
#    - B = guardar y salir
#  Los valores se guardan en el robot (cfg.PARAMS_FILE) y se cargan al encender; lo guardado
#  manda sobre config.py y pista.py. Si el CyberPi no permite escribir archivos, los cambios duran
#  hasta apagarlo: anotarlos en config.py o pista.py.

import json
import hal
import config as cfg
import motion
import gripper

ACTIONS = ["Calibrar linea", "Ver sensores", "Probar giro 90", "Probar recta 50",
           "Restaurar valores"]
DEFAULTS = {}


## @brief Guarda los valores de fábrica de cfg.P (los de config.py y pista.py).
#  @details Se llama al arrancar, antes de load(); la acción "Restaurar valores" vuelve a ellos.
def remember_defaults():
    for k in cfg.P:
        DEFAULTS[k] = cfg.P[k]


## @brief Copia de los parámetros y la calibración de línea actuales.
#  @return Dict {"P": {...}, "LINE_CAL": {...}} listo para guardar como JSON.
def snapshot():
    cal = {}
    for pr in cfg.PROBES:
        cal[pr] = [cfg.LINE_CAL[pr][0], cfg.LINE_CAL[pr][1]]
    params = {}
    for k in cfg.P:
        params[k] = cfg.P[k]
    return {"P": params, "LINE_CAL": cal}


## @brief Aplica un diccionario guardado.
#  @details Ignora claves desconocidas (archivos de versiones viejas del programa).
#  @param data Dict como el de snapshot().
def apply(data):
    for k in data.get("P", {}):
        if k in cfg.P:
            cfg.P[k] = data["P"][k]
    cal = data.get("LINE_CAL", {})
    for pr in cal:
        if pr in cfg.LINE_CAL:
            cfg.LINE_CAL[pr] = [cal[pr][0], cal[pr][1]]


## @brief Guarda los parámetros en el robot (cfg.PARAMS_FILE).
#  @return True si se pudo escribir el archivo.
def save():
    try:
        f = open(cfg.PARAMS_FILE, "w")
        f.write(json.dumps(snapshot()))
        f.close()
        return True
    except Exception:
        return False


## @brief Carga los parámetros guardados en el robot.
#  @return True si había archivo y se aplicó; False = se usan los de fábrica.
def load():
    try:
        f = open(cfg.PARAMS_FILE)
        data = json.loads(f.read())
        f.close()
        apply(data)
        return True
    except Exception:
        return False


# ---------------------------------------------------------------- calibración de línea
## @brief Calibración de cada sonda a partir de lecturas de luz (función pura).
#  @param samples Dict sonda -> lista de lecturas de luz tomadas pasando sobre la línea.
#  @return Dict sonda -> [blanco, negro], o None si alguna sonda no vio contraste (no pasó
#  sobre la línea).
def cal_from_samples(samples):
    out = {}
    for pr in cfg.PROBES:
        s = samples[pr]
        hi = max(s)
        lo = min(s)
        if hi - lo < 15:
            return None
        out[pr] = [hi, lo]
    return out


## @brief Calibra las sondas de línea (acción "Calibrar linea").
#  @details Con el robot centrado sobre la línea, gira ~45° a cada lado leyendo las 4 sondas y
#  guarda en cfg.LINE_CAL el blanco y el negro de cada una.
#  @return True si todas las sondas vieron contraste.
def calibrate_line():
    hal.show("Calibrando linea...", line=2)
    samples = {}
    for pr in cfg.PROBES:
        samples[pr] = []
    start = hal.heading()
    v = cfg.P["v_turn_min"] * 1.3
    for target, d in ((45, 1), (-45, -1), (0, 1)):
        t0 = hal.now()
        while hal.now() - t0 < 4:
            h = hal.heading() - start
            if (d > 0 and h >= target) or (d < 0 and h <= target):
                break
            hal.drive(d * v, -d * v)
            for pr in cfg.PROBES:
                samples[pr].append(hal.probe_light(pr))
            hal.wait(cfg.LOOP_DT)
    hal.stop()
    cal = cal_from_samples(samples)
    if cal is None:
        hal.show("Sin contraste: centrar", line=2)
        return False
    for pr in cal:
        cfg.LINE_CAL[pr] = cal[pr]
    hal.show("Linea OK", line=2)
    return True


## @brief Muestra oscuridad por sonda, offset, pitch y rumbo hasta presionar B.
def show_sensors():
    hal.wait(0.3)
    while not hal.button("b"):
        dk = hal.darkness()
        hal.show("%.2f %.2f %.2f %.2f" % (dk["L2"], dk["L1"], dk["R1"], dk["R2"]), line=1)
        hal.show("offset %d" % hal.offset_from(dk), line=2)
        hal.show("pitch %d  rumbo %d" % (hal.pitch(), hal.heading()), line=3)
        hal.wait(0.05)
    hal.wait(0.3)


## @brief Ejecuta una acción del menú.
#  @param name Nombre de la acción (ver ACTIONS).
#  @return True si la acción terminó bien.
def run_action(name):
    if name == "Calibrar linea":
        return calibrate_line()
    if name == "Ver sensores":
        show_sensors()
        return True
    if name == "Probar giro 90":
        return motion.turn(90)
    if name == "Probar recta 50":
        return motion.straight(50)
    if name == "Restaurar valores":
        for k in DEFAULTS:
            cfg.P[k] = DEFAULTS[k]
        return True
    return False


# ---------------------------------------------------------------- menú
## @brief Lista de renglones del menú: primero los parámetros (cfg.TUNE), luego las acciones.
#  @return Lista de ("param", spec) o ("action", nombre).
def items():
    out = []
    for t in cfg.TUNE:
        out.append(("param", t))
    for a in ACTIONS:
        out.append(("action", a))
    return out


## @brief Cambia un parámetro un paso.
#  @param spec (grupo, clave, paso, mínimo, máximo), de cfg.TUNE.
#  @param direction 1 = subir, -1 = bajar.
#  @return El valor nuevo (recortado al rango).
def bump(spec, direction):
    key = spec[1]
    v = cfg.P[key] + direction * spec[2]
    v = hal.clamp(v, spec[3], spec[4])
    if isinstance(spec[2], float):
        v = round(v, 3)
    else:
        v = int(v)
    cfg.P[key] = v
    return v


## @brief Prueba física de un parámetro (botón A).
#  @details Alturas "h_": el brazo va a esa altura; aperturas "claw_": la garra va a esa
#  apertura; tabla del brazo "arm_": el brazo va a la altura de transporte.
#  @param spec Renglón de cfg.TUNE.
def try_param(spec):
    key = spec[1]
    if key.startswith("h_"):
        gripper.arm_to(cfg.P[key])
    elif key.startswith("claw_"):
        gripper.claw(cfg.P[key])
    elif key.startswith("arm_"):
        gripper.arm_to(cfg.P["h_carry"])


## @brief Bucle del menú de ajustes; sale al presionar B (guarda antes).
def menu():
    lst = items()
    i = 0
    hal.wait(0.3)
    while True:
        kind, it = lst[i]
        if kind == "param":
            hal.show("%s  %d/%d" % (it[0], i + 1, len(lst)), line=0)
            hal.show("%s = %s" % (it[1], cfg.P[it[1]]), line=1, size=16)
            hal.show("<- -> valor  A prueba", line=2)
        else:
            hal.show("Accion  %d/%d" % (i + 1, len(lst)), line=0)
            hal.show(it, line=1, size=16)
            hal.show("A = ejecutar", line=2)
        hal.show("B = guardar y salir", line=3)
        if hal.button("down"):
            i = (i + 1) % len(lst)
            hal.wait(0.2)
        elif hal.button("up"):
            i = (i - 1) % len(lst)
            hal.wait(0.2)
        elif kind == "param" and hal.button("right"):
            bump(it, 1)
            hal.wait(0.15)
        elif kind == "param" and hal.button("left"):
            bump(it, -1)
            hal.wait(0.15)
        elif hal.button("a"):
            if kind == "param":
                try_param(it)
            else:
                run_action(it)
            hal.wait(0.3)
        elif hal.button("b"):
            ok = save()
            hal.show("Guardado" if ok else "No se pudo guardar", line=3)
            hal.wait(0.8)
            hal.clear_abort()
            return
        hal.wait(0.02)

