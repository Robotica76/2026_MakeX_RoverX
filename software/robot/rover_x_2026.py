# Rover X 2026: programa de competencia (mBot2 + kit Rover, MicroPython en el CyberPi).
#
# Flujo:
#   encender -> menú de nivel (se sugiere por el color de la zona; Arr/Abj cambia)
#   A (señal del juez) -> autónomo desde ese nivel   [B durante el autónomo = abortar]
#   al terminar: A = control manual (alianza)  ·  B = volver al menú (reinicio)
#   en manual: B en el CyberPi = volver al menú
#
# Para cargarlo en mBlock: python3 tools/scripts/bundle_for_mblock.py software/robot/rover_x_2026.py
# y pegar build/rover_x_2026_mblock.py en el editor de Python (modo Upload).

import hal
import autonomous as auto
import manual
import gripper


def after_auto_menu():
    hal.clear_abort()
    hal.wait(0.4)
    while True:
        hal.show("A=Manual  B=Menu", line=4)
        if hal.button("a"):
            return "manual"
        if hal.button("b"):
            return "menu"
        hal.wait(0.02)


def main():
    hal.init()
    gripper.home()
    while True:
        level = auto.choose_level()
        hal.clear_abort()
        hal.show("AUTONOMO nivel %d" % level, line=0)
        points = auto.run(level)
        hal.show("Fin auto: ~%d pts" % points, line=0)
        if after_auto_menu() == "manual":
            hal.wait(0.4)
            manual.loop()
        hal.stop()
        hal.wait(0.4)


main()
