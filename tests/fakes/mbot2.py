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
