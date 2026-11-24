# Rover X 2026: programa AUTÓNOMO (mBot2 + kit Rover, MicroPython en el CyberPi).
#
# Botones del CyberPi:
#   joystick   arriba = nivel 1, abajo = 2, izquierda = 3, derecha = 4 (cfg.LEVEL_KEYS)
#   B          empezar: corre el tramo de ese nivel y se detiene al terminarlo (cfg.START_BUTTON)
#   A          parar el robot durante el tramo (cfg.ABORT_BUTTON)
#   centro     en la pantalla de inicio: menú de ajustes (cfg.MENU_BUTTON)
# Al terminar o parar vuelve a la pantalla de inicio con el nivel que sigue ya elegido.
#
# El control manual es otro programa: rover_x_manual.py.
# Archivo listo para mBlock: mblock/rover_x_auto.py
# (se regenera con: python3 tools/scripts/bundle_for_mblock.py --all).

import hal
import config as cfg
import autonomous as auto
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
        hal.clear_abort()
        hal.show("AUTONOMO nivel %d" % level, line=0)
        points = auto.run(level, skip=auto.completed)
        hal.stop()
        if hal.abort_requested():
            hal.show("PARADO con %s" % cfg.ABORT_BUTTON.upper(), line=3)
        else:
            hal.show("Fin tramo: ~%d pts" % points, line=3)
        level = auto.next_level if auto.next_level <= 4 else None
        hal.clear_abort()
        hal.wait(0.4)


main()
