# Rover X 2026: programa MANUAL (control a distancia con el gamepad, misiones de alianza).
#
# Botones del CyberPi:
#   B   empezar el control manual (cfg.START_BUTTON)
#   A   parar: detiene el robot y vuelve a la espera (cfg.ABORT_BUTTON)
# Los mandos del gamepad están en docs/control_manual.md.
#
# El autónomo es otro programa: rover_x_auto.py.
# Archivo listo para mBlock: mblock/rover_x_manual.py
# (se regenera con: python3 tools/scripts/bundle_for_mblock.py --all).

import hal
import config as cfg
import manual
import gripper
import settings


def wait_start():
    """Espera B (y que se suelte, para que no cuente como otra pulsación)."""
    hal.show("MANUAL", line=0, size=24)
    hal.show("%s = empezar" % cfg.START_BUTTON.upper(), line=1)
    hal.show("%s = parar" % cfg.ABORT_BUTTON.upper(), line=2)
    hal.show("gamepad: %s" % ("listo" if hal.pad_ok() else "NO conectado"), line=3)
    while not hal.button(cfg.START_BUTTON):
        hal.wait(0.02)
    while hal.button(cfg.START_BUTTON):
        hal.wait(0.02)


def main():
    settings.remember_defaults()
    settings.load()
    hal.init()
    gripper.home()
    while True:
        wait_start()
        manual.loop()            # termina con A
        hal.stop()
        while hal.button(cfg.ABORT_BUTTON):
            hal.wait(0.02)
        hal.clear_abort()


main()
