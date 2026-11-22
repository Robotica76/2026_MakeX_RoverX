"""cyberpi falso para pruebas en PC (ver world.py)."""
import world


class _Timer:
    def get(self):
        return world.t

    def reset(self):
        world.t = 0.0


class _Controller:
    def is_press(self, name):
        return name in world.pressed


class _Display:
    def show_label(self, text, size=16, x=0, y=0, index=0):
        world.labels[index] = str(text)


class _Console:
    def println(self, text):
        pass

    def print(self, text):
        pass

    def clear(self):
        pass

    def set_font(self, size):
        pass


class _Led:
    def on(self, r, g, b, id="all"):
        world.leds = (r, g, b)


timer = _Timer()
controller = _Controller()
display = _Display()
console = _Console()
led = _Led()


def get_rotation(axis):
    return world.rot_z if axis == "z" else 0.0


def reset_rotation(axis="all"):
    world.rot_z = 0.0


def get_yaw():
    return world.rot_z


def get_pitch():
    return world.sensors["pitch"]()


def get_roll():
    return 0.0
