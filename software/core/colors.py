# Rover X 2026: reconocimiento de colores (zonas de inicio, marcas verdes, tarjetas).

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


def nearest(rgb, refs):
    """Nombre del color de referencia más cercano (distancia sobre cromaticidad + brillo).

    Normalizar por el brillo hace la clasificación menos sensible a cambios de luz
    (reglamento: el robot debe adaptarse a la iluminación)."""
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


def normalize(name):
    """Nombres del sensor -> nombres del programa (el sensor puede devolver mayúsculas)."""
    if name is None:
        return "unknown"
    return str(name).strip().lower()


def read(probe):
    if cfg.P["color_mode"] == 1:
        return nearest(hal.probe_rgb(probe), cfg.COLOR_REF)
    return normalize(hal.probe_color_builtin(probe))


def read_stable(probe, n=None):
    """Lee hasta tener n lecturas iguales seguidas (máx. 4n intentos)."""
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


def show(name):
    rgb = LED_RGB.get(name, (40, 40, 40))
    hal.leds(rgb[0], rgb[1], rgb[2])
    hal.show(name, line=3, size=24)


def zone_level(color):
    """Nivel de inicio según el color de la zona en la que está el robot (0 = no se sabe)."""
    return cfg.ZONE_LEVEL.get(color, 0)


def is_card_color(name):
    """Una tarjeta es cualquier color que no sea el piso blanco ni la línea negra."""
    return name not in ("white", "black", "unknown", "")
