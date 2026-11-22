# Rover X 2026: prueba de API y calibración en el robot (programa independiente).
#
# Cargarlo tal cual en mBlock (modo Upload). Sirve para confirmar las funciones de
# docs/api_mbot2.md y llenar los valores "CALIBRAR" de software/core/config.py.
#
# Joystick del CyberPi: arriba/abajo cambia de página. A ejecuta la prueba de la página.
#   0 Línea   : offset, get_line_sta y luz por sonda. Poner UNA sonda sobre negro para leer su
#               bit; con la línea entre dos sondas, ver cuánto baja la luz de cada una.
#   1 Colores : nombre (get_color_sta) y RGB de cada sonda, para COLOR_REF.
#   2 IMU     : rotación z, yaw, pitch. Girar a la derecha / levantar la nariz y ver el signo.
#   3 Motores : A = drive_power(40, -40) 1 s. Debe AVANZAR. Muestra grados del encoder.
#   4 Recta   : A = mbot2.straight(50). Medir con cinta y leer grados -> CM_PER_DEG.
#   5 Servos  : A alterna s1/s2 entre 60 y 150 grados.
#   6 Sonar   : distancia del ultrasónico 2.
#   7 Gamepad : teclas y joysticks del Bluetooth Controller.

import time
import cyberpi
import mbot2
import mbuild

try:
    import gamepad
    HAS_PAD = True
except ImportError:
    HAS_PAD = False

q = mbuild.quad_rgb_sensor
PAGES = ["Linea", "Colores", "IMU", "Motores", "Recta", "Servos", "Sonar", "Gamepad"]
page = 0
servo_hi = False


def label(i, text):
    cyberpi.display.show_label(text, 12, 0, 14 * i, index=i)


def enc():
    try:
        return "%d %d" % (mbot2.EM_get_angle("EM1"), mbot2.EM_get_angle("EM2"))
    except Exception as e:
        return "ERR " + str(e)


def show_page():
    p = PAGES[page]
    label(0, "%d %s  (A=probar)" % (page, p))
    if p == "Linea":
        sta = q.get_line_sta("all", 1)
        label(1, "offset %s" % q.get_offset_track(1))
        label(2, "line_sta %d  bin %s" % (sta, bin(sta)))
        label(3, "L2%d L1%d R1%d R2%d" % (q.is_line("L2", 1), q.is_line("L1", 1),
                                          q.is_line("R1", 1), q.is_line("R2", 1)))
        label(4, "luz %d %d %d %d" % (q.get_light("L2"), q.get_light("L1"),
                                      q.get_light("R1"), q.get_light("R2")))
    elif p == "Colores":
        for i, pr in enumerate(("L2", "L1", "R1", "R2")):
            label(1 + i, "%s %s %d,%d,%d" % (pr, q.get_color_sta(pr), q.get_red(pr),
                                            q.get_green(pr), q.get_blue(pr)))
    elif p == "IMU":
        label(1, "rot z %.1f" % cyberpi.get_rotation("z"))
        label(2, "yaw %.1f" % cyberpi.get_yaw())
        label(3, "pitch %.1f" % cyberpi.get_pitch())
        label(4, "roll %.1f" % cyberpi.get_roll())
    elif p in ("Motores", "Recta"):
        label(1, "encoders " + enc())
        label(2, "")
        label(3, "")
        label(4, "")
    elif p == "Servos":
        label(1, "s1/s2 alto=%s" % servo_hi)
        label(2, "")
        label(3, "")
        label(4, "")
    elif p == "Sonar":
        label(1, "dist %s cm" % mbuild.ultrasonic2.get(index=1))
        label(2, "")
        label(3, "")
        label(4, "")
    elif p == "Gamepad":
        if not HAS_PAD:
            label(1, "sin modulo gamepad")
            return
        keys = [k for k in ("Up", "Down", "Left", "Right", "N1", "N2", "N3", "N4",
                            "L1", "L2", "R1", "R2", "Start", "Select")
                if gamepad.is_key_pressed(k)]
        label(1, "Lx %s Ly %s" % (gamepad.get_joystick("Lx"), gamepad.get_joystick("Ly")))
        label(2, "Rx %s Ry %s" % (gamepad.get_joystick("Rx"), gamepad.get_joystick("Ry")))
        label(3, " ".join(keys))
        label(4, "")


def run_test():
    global servo_hi
    p = PAGES[page]
    if p == "Motores":
        try:
            mbot2.EM_reset_angle("all")
        except Exception:
            pass
        mbot2.drive_power(40, -40)
        time.sleep(1)
        mbot2.drive_power(0, 0)
    elif p == "Recta":
        try:
            mbot2.EM_reset_angle("all")
        except Exception:
            pass
        mbot2.straight(50)
    elif p == "Servos":
        servo_hi = not servo_hi
        a = 150 if servo_hi else 60
        mbot2.servo_set(a, "s1")
        mbot2.servo_set(a, "s2")
    elif p == "IMU":
        cyberpi.reset_rotation(axis="all")


q.set_led(color="white")
while True:
    if cyberpi.controller.is_press("up"):
        page = (page + 1) % len(PAGES)
        time.sleep(0.3)
    elif cyberpi.controller.is_press("down"):
        page = (page - 1) % len(PAGES)
        time.sleep(0.3)
    elif cyberpi.controller.is_press("a"):
        run_test()
        time.sleep(0.3)
    show_page()
    time.sleep(0.05)
