# Rover X 2026: programa de competencia (mBot2 + kit Rover, MicroPython en el CyberPi).
#
# Flujo (por TRAMOS, como lo usa el equipo):
#   encender -> carga los parámetros guardados -> pantalla de inicio
#   joystick: arriba = nivel 1, abajo = 2, izquierda = 3, derecha = 4 (cfg.LEVEL_KEYS)
#   B (play, cfg.START_BUTTON) -> corre el tramo de ese nivel y se detiene al terminarlo
#   [A durante el autónomo = abortar]
#   el equipo pide reinicio, coloca el robot en la zona, elige el nivel (ya sugerido) y B
#   en la pantalla de inicio: A = control manual (alianza), centro del joystick = ajustes
#
# Para cargarlo en mBlock: python3 tools/scripts/bundle_for_mblock.py software/robot/rover_x_2026.py
# y pegar build/rover_x_2026_mblock.py en el editor de Python (modo Upload).

import hal
import autonomous as auto
import manual
import gripper
import settings


def main():
    settings.remember_defaults()
    loaded = settings.load()
    hal.init()
    hal.show("Parametros: %s" % ("guardados" if loaded else "de fabrica"), line=3)
    gripper.home()
    level = None
    while True:
        level = auto.choose_level(level)
        if level == 0:
            settings.menu()
            level = None
            continue
        if level < 0:
            hal.wait(0.4)
            manual.loop()
            hal.stop()
            level = None
            continue
        hal.clear_abort()
        hal.show("AUTONOMO nivel %d" % level, line=0)
        points = auto.run(level, skip=auto.completed)
        hal.show("Fin tramo: ~%d pts" % points, line=3)
        level = auto.next_level if auto.next_level <= 4 else None
        hal.stop()
        hal.wait(0.4)


main()
