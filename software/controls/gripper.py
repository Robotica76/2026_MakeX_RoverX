## @file gripper.py
#  @brief Garra (servo de cierre) y brazo (servo de altura).
#  @details El brazo se manda por ALTURA en cm (geometry.arm_servo convierte a ángulo del servo),
#  así que al cambiar el brazo solo se mide de nuevo la tabla arm_h* / arm_x* (config.py).

import hal
import config as cfg
import geometry as geo


## @brief Mueve un servo y espera a que llegue (cfg.SERVO_STEP_S).
#  @param port Puerto del servo.
#  @param angle Ángulo en grados.
def _move(port, angle):
    hal.servo(port, angle)
    hal.wait(cfg.SERVO_STEP_S)


## @brief Mueve la garra a una apertura.
#  @param angle Ángulo del servo de la garra (claw_open = abierta, claw_cube = cerrada en el cubo).
def claw(angle):
    _move(cfg.SERVO_CLAW, angle)


## @brief Abre la garra (cfg.P["claw_open"]).
def open_claw():
    claw(cfg.P["claw_open"])


## @brief Sube o baja el brazo hasta que la garra quede a una altura.
#  @param h Altura del centro de la garra sobre el piso (cm).
def arm_to(h):
    _move(cfg.SERVO_ARM, geo.arm_servo(h))


## @brief Posición de viaje: brazo arriba (h_top) y garra abierta.
#  @details Garra sobre la nariz: lo más compacto para colocar el robot en las zonas de inicio,
#  que son más chicas que él.
def home():
    arm_to(cfg.P["h_top"])
    open_claw()


## @brief Sostiene el objeto quieto el tiempo que pide el reglamento (>= 5 s).
#  @param seconds Segundos (por defecto cfg.HOLD_VALID_S).
#  @return True si terminó; False si se presionó A (parar).
def hold(seconds=None):
    if seconds is None:
        seconds = cfg.HOLD_VALID_S
    hal.stop()
    t0 = hal.now()
    while hal.now() - t0 < seconds:
        if hal.abort_requested():
            return False
        hal.show("Sosteniendo %d" % int(seconds - (hal.now() - t0) + 1), line=2)
        hal.wait(0.1)
    return True
