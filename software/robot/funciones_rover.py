# Funciones de movimiento preciso del Rover X (versión mejorada de FuncionesEnPython.mblock).
#
# Mismo uso que el programa original: se sube desde mBlock (modo "subir", Python) y se prueba con
# los botones. Lo que cambia:
#   - avanzar_cm(): avanza por DISTANCIA (encoders) y no por tiempo, sosteniendo el rumbo con el
#     giroscopio. Usa las medidas del robot: diámetro efectivo de la oruga 4.18 cm y la oruga
#     izquierda avanza 2.3 % menos por grado (prueba de 1 m: 63.5 / 65 cm).
#   - girar(): en vez de parar al 77 % del ángulo ("* 1.3 por error del sensor"), FRENA al
#     acercarse y termina corrigiendo hasta quedar a menos de 1°. Si el error era la inercia,
#     esto lo resuelve; si el giroscopio de verdad mide de menos, se corrige con GYRO_SCALE
#     (ver la prueba "giro a mano" abajo).
#   - El rumbo no se reinicia en cada giro: los ángulos se suman sin acumular error.
#
# Botones:
#   A          secuencia de prueba (la misma del programa original, ahora en cm)
#   B          detener; con el robot quieto, pone el rumbo en 0 (para la prueba a mano)
#   joystick   arriba 45° der, abajo 90° izq, izquierda 30° der, derecha 180°, centro 360°
# En reposo la pantalla muestra el rumbo: GIRAR EL ROBOT A MANO 360° (contra una escuadra o una
# marca en el piso) y anotar lo que marca -> GYRO_SCALE = 360 / lectura.

import event, time, math, cyberpi, mbot2

# ---------------------------------------------------------------- medidas y ajustes
RUEDA_D = 4.18        # CALIBRADO (1 m): diámetro efectivo de la oruga, cm
LR = 0.977            # CALIBRADO (1 m): cm por grado izquierda / derecha (63.5 / 65)
GYRO_SCALE = 1.0      # TODO(calibrar con la prueba a mano): grados reales por grado leído
KP_RECTO = 1.5        # rpm de corrección por grado de desvío al avanzar
KP_GIRO = 1.2         # rpm por grado que falta al girar
GIRO_MIN = 12         # TODO(ajustar) rpm mínimas para que las orugas giren en el lugar
TOL_GIRO = 1.0        # grados: el giro termina con menos de esto de error
RAMPA_S = 0.4         # s para llegar a la velocidad pedida (sin tirones: no patina)

CM_POR_GRADO = math.pi * RUEDA_D / 360.0
K_IZQ = CM_POR_GRADO * 2 * LR / (1 + LR)     # cm por grado de cada oruga (su promedio es
K_DER = CM_POR_GRADO * 2 / (1 + LR)          # CM_POR_GRADO)

ejecutar = False
detener_ejecucion = False
rumbo_cero = 0.0


# ---------------------------------------------------------------- motores y sensores
def motores(izq, der):
    """rpm de cada oruga, + = adelante (EM2 va al revés)."""
    mbot2.EM_set_speed(izq, "EM1")
    mbot2.EM_set_speed(-der, "EM2")


def parar():
    mbot2.EM_stop("EM1")
    mbot2.EM_stop("EM2")


def rumbo():
    """Grados acumulados, + = giro a la DERECHA (como el ángulo de girar())."""
    return -cyberpi.get_rotation('z') * GYRO_SCALE - rumbo_cero


def orugas_cm():
    """(izquierda, derecha) en cm desde el arranque, + = adelante."""
    return mbot2.EM_get_angle("EM1") * K_IZQ, -mbot2.EM_get_angle("EM2") * K_DER


def limitar(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# ---------------------------------------------------------------- movimientos
def mover_motores(vel_m1, vel_m2, duracion):
    """Movimiento por tiempo (el del programa original; se deja para pruebas manuales)."""
    mbot2.EM_set_speed(vel_m1, "EM1")
    mbot2.EM_set_speed(-vel_m2, "EM2")
    inicio = time.time()
    while time.time() - inicio < duracion:
        if detener_ejecucion:
            parar()
            return False
        time.sleep(0.05)
    parar()
    return True


def avanzar_cm(cm, rpm=50, rumbo_obj=None):
    """Avanza `cm` (negativo = atrás) por encoders, sosteniendo el rumbo con el giroscopio.
    rumbo_obj: rumbo a sostener (por omisión, el de ahora). Arranca y frena suave."""
    if rumbo_obj is None:
        rumbo_obj = rumbo()
    signo = 1 if cm >= 0 else -1
    meta = abs(cm)
    l0, r0 = orugas_cm()
    inicio = time.time()
    while True:
        if detener_ejecucion:
            parar()
            return False
        l, r = orugas_cm()
        hecho = abs(((l - l0) + (r - r0)) / 2.0)
        falta = meta - hecho
        if falta <= 0.3:
            break
        v = rpm
        t = time.time() - inicio
        if t < RAMPA_S:                                  # arranque suave
            v = rpm * max(0.3, t / RAMPA_S)
        if falta < 6:                                    # frenar los últimos 6 cm
            v = max(15, rpm * falta / 6.0)
        # el robot gira según (izquierda - derecha), avance o retroceda: la misma corrección
        corr = limitar(KP_RECTO * (rumbo_obj - rumbo()), -rpm * 0.5, rpm * 0.5)
        # la derecha avanza más por grado: va un poco más lenta para ir recto (LR)
        motores(signo * v + corr, (signo * v - corr) * LR)
        if time.time() - inicio > 2 + meta / 3.0:        # seguridad: atorado
            break
        time.sleep(0.01)
    parar()
    return True


def girar(angulo, rpm_max=40, timeout=6):
    """Gira `angulo` grados en el lugar (+ = derecha) con el giroscopio: frena al acercarse
    y corrige hasta quedar a menos de TOL_GIRO. No reinicia el giroscopio."""
    meta = rumbo() + angulo
    quieto = 0
    inicio = time.time()
    while time.time() - inicio < timeout:
        if detener_ejecucion:
            parar()
            return False
        err = meta - rumbo()
        cyberpi.display.show_label(str(int(round(err))), 16, "center")
        if abs(err) < TOL_GIRO:
            parar()
            quieto += 1
            if quieto >= 5:                              # 50 ms sin moverse fuera de tolerancia
                return True
        else:
            quieto = 0
            v = limitar(KP_GIRO * abs(err), GIRO_MIN, rpm_max)
            if err > 0:
                motores(v, -v)                           # derecha: izquierda adelante
            else:
                motores(-v, v)
        time.sleep(0.01)
    parar()
    return False


def girar_a(rumbo_obj, rpm_max=40):
    """Gira hasta el rumbo absoluto `rumbo_obj` (0 = como se encendió o se puso en 0 con B)."""
    err = rumbo_obj - rumbo()
    while err > 180:
        err -= 360
    while err < -180:
        err += 360
    return girar(err, rpm_max)


# ---------------------------------------------------------------- pantalla
def mensaje(texto, tam=12, espera=0.0):
    cyberpi.display.clear()
    cyberpi.display.show_label(texto, tam, "center")
    if espera:
        time.sleep(espera)


def menu():
    cyberpi.display.clear()
    cyberpi.display.show_label("A=Prueba B=Stop", 10, "top")


def correr(nombre, accion):
    """Corre una prueba si no hay otra corriendo."""
    global ejecutar, detener_ejecucion
    if ejecutar:
        return
    ejecutar = True
    detener_ejecucion = False
    mensaje(nombre, 10, 0.5)
    ok = accion()
    parar()
    ejecutar = False
    mensaje("OK" if ok else "DETENIDO", 12, 0.5)
    menu()


# ==============================================
# Al encender el robot
# ==============================================
@event.start
def on_start():
    mensaje("ROVER LISTO", 14, 1)
    menu()


# ==============================================
# BOTON A - SECUENCIA DE PRUEBA
# ==============================================
# La del programa original: a 50 rpm la oruga avanza ~10.9 cm/s, así que 4 s = 44 cm,
# 8 s = 88 cm y 2 s = 22 cm. Cada tramo recto sostiene el rumbo con el que terminó el giro.
def secuencia():
    pasos = [lambda: avanzar_cm(44), lambda: girar(-90), lambda: avanzar_cm(88),
             lambda: avanzar_cm(-44), lambda: girar(-90), lambda: avanzar_cm(22),
             lambda: girar(90), lambda: avanzar_cm(44)]
    for paso in pasos:
        if not paso():
            return False
    return True


@event.is_press('a')
def test_secuencia():
    correr("SECUENCIA", secuencia)


# ==============================================
# JOYSTICK - GIROS DE PRUEBA
# ==============================================
@event.is_press('up')
def girar_45_derecha():
    correr("GIRAR 45 DER", lambda: girar(45, 35))


@event.is_press('down')
def girar_90_izquierda():
    correr("GIRAR 90 IZQ", lambda: girar(-90, 35))


@event.is_press('left')
def girar_30_derecha():
    correr("GIRAR 30 DER", lambda: girar(30, 30))


@event.is_press('right')
def girar_180():
    correr("GIRAR 180", lambda: girar(180, 45))


@event.is_press('middle')
def girar_360():
    correr("GIRAR 360", lambda: girar(360, 50))


# ==============================================
# BOTON B - DETENER / RUMBO EN 0
# ==============================================
@event.is_press('b')
def detener():
    global detener_ejecucion, rumbo_cero
    if ejecutar:
        detener_ejecucion = True
        parar()
    else:
        # robot quieto: rumbo en 0 para la prueba a mano (girarlo 360° y leer la pantalla)
        rumbo_cero = -cyberpi.get_rotation('z') * GYRO_SCALE
        mensaje("RUMBO = 0", 12, 0.5)
        menu()


# ==============================================
# PROGRAMA PRINCIPAL: en reposo muestra el rumbo (prueba a mano del giroscopio)
# ==============================================
@event.start
def main():
    while True:
        if not ejecutar:
            cyberpi.display.show_label("rumbo %d" % int(round(rumbo())), 16, "center")
        time.sleep(0.2)
