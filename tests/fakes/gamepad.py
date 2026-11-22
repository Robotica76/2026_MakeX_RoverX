"""gamepad falso para pruebas en PC (ver world.py)."""
import world


def is_key_pressed(key):
    return key in world.pad


def get_joystick(axis):
    return world.axes.get(axis, 0)
