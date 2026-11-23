"""mbot2 falso para pruebas en PC (ver world.py)."""
import world


def drive_power(em1, em2):
    world.em = [float(em1), float(em2)]


def drive_speed(em1, em2):
    drive_power(em1, em2)


def EM_stop(port="all"):
    world.em = [0.0, 0.0]


def EM_get_angle(port):
    return world.enc[0] if port == "EM1" else world.enc[1]


def EM_reset_angle(port="all"):
    world.enc = [0.0, 0.0]


def servo_set(angle, port):
    world.servos[port] = angle


def servo_get(port):
    return world.servos.get(port, 90)


def straight(cm):
    pass


def turn(deg):
    pass


def EM_set_speed(rpm, port):
    """rpm de un motor -> potencia del mundo falso (oruga de 4.18 cm, 46 cm/s al 100 %)."""
    p = float(rpm) * 3.14159 * 4.18 / 60.0 / 46.0 * 100.0
    if port == "EM1":
        world.em = [p, world.em[1]]
    else:
        world.em = [world.em[0], p]
