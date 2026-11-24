## @file colors.py
#  @brief Reconocimiento de colores: zonas de inicio, marcas verdes y tarjetas.
#  @details Dos modos (cfg.P["color_mode"]): 0 = el nombre que da el propio sensor
#  (get_color_sta); 1 = el color de referencia RGB más cercano (cfg.COLOR_REF).

import hal
import config as cfg

# Color para LEDs del CyberPi (misión 11: mostrar el color reconocido)
LED_RGB = {
    "black": (0, 0, 0),
    "white": (255, 255, 255),
    "red": (255, 0, 0),
    "yellow": (255, 200, 0),
    "green": (0, 255, 0),
    "blue": (0, 0, 255),
    "cyan": (0, 255, 255),
    "purple": (180, 0, 255),
}


## @brief Color de referencia más cercano a una lectura RGB.
#  @details Distancia sobre cromaticidad + brillo. Normalizar por el brillo hace la clasificación
#  menos sensible a cambios de luz (reglamento: el robot debe adaptarse a la iluminación).
#  @param rgb (rojo, verde, azul) leídos por la sonda.
#  @param refs Dict nombre -> (r, g, b) de referencia (cfg.COLOR_REF).
#  @return Nombre del color más cercano.
def nearest(rgb, refs):
    r, g, b = rgb
    total = r + g + b + 1
    best = None
    best_d = None
    for name in refs:
        rr, rg, rb = refs[name]
        rt = rr + rg + rb + 1
        dc = (r / total - rr / rt) ** 2 + (g / total - rg / rt) ** 2 + (b / total - rb / rt) ** 2
        db = ((total - rt) / 765.0) ** 2
        d = dc * 4 + db
        if best_d is None or d < best_d:
            best = name
            best_d = d
    return best


## @brief Nombre del sensor -> nombre del programa.
#  @param name Nombre que devuelve el sensor (puede venir en mayúsculas o ser None).
#  @return Nombre en minúsculas, o "unknown".
def normalize(name):
    if name is None:
        return "unknown"
    return str(name).strip().lower()


## @brief Color que ve una sonda (según color_mode).
#  @param probe Nombre de la sonda.
#  @return Nombre del color ("red", "blue", ...).
def read(probe):
    if cfg.P["color_mode"] == 1:
        return nearest(hal.probe_rgb(probe), cfg.COLOR_REF)
    return normalize(hal.probe_color_builtin(probe))


## @brief Lee hasta tener n lecturas iguales seguidas.
#  @param probe Nombre de la sonda.
#  @param n Lecturas iguales necesarias (por defecto cfg.COLOR_CONFIRM); máx. 4n intentos.
#  @return El color confirmado, o la última lectura si no se confirmó.
def read_stable(probe, n=None):
    if n is None:
        n = cfg.COLOR_CONFIRM
    last = None
    count = 0
    for _ in range(n * 4):
        c = read(probe)
        if c == last:
            count += 1
        else:
            last = c
            count = 1
        if count >= n:
            return c
        hal.wait(cfg.LOOP_DT)
    return last


## @brief Muestra un color en los LEDs y la pantalla (misión 11).
#  @param name Nombre del color.
def show(name):
    rgb = LED_RGB.get(name, (40, 40, 40))
    hal.leds(rgb[0], rgb[1], rgb[2])
    hal.show(name, line=3, size=24)


## @brief Nivel de inicio según el color de la zona.
#  @param color Color de la zona en la que está el robot.
#  @return 1..4 (cfg.ZONE_LEVEL), 0 = no se sabe.
def zone_level(color):
    return cfg.ZONE_LEVEL.get(color, 0)


## @brief ¿Es color de tarjeta?
#  @details Una tarjeta es cualquier color que no sea el piso blanco ni la línea negra.
#  @param name Nombre del color.
#  @return True si puede ser una tarjeta.
def is_card_color(name):
    return name not in ("white", "black", "unknown", "")
