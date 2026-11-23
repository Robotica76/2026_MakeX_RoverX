# calibrations: programa de prueba de calibración del Rover X (movimiento lineal, giros,
# círculo y giroscopio). Se sube desde mBlock (Python, modo "subir") como el programa del equipo.
#
# BOTÓN A: secuencia completa (en un espacio libre de ~1 m x 1 m, con una marca de salida):
#   1. avanza 20 cm            2. gira 90° a la izquierda   3. avanza 10 cm
#   4. gira 90° a la derecha   5. gira 90° a la izquierda   6. avanza 20 cm
#   7. dibuja un círculo de 40 cm de diámetro (hacia la izquierda) y para
#   Al terminar cada paso la pantalla muestra lo MEDIDO por el robot (encoders y giroscopio).
#   Medir en el piso con regla/transportador y comparar (hoja abajo).
# JOYSTICK (pruebas sueltas):
#   arriba = avanza 50 cm   abajo = gira 360° en el lugar   izquierda = gira 90° izq
#   derecha = gira 90° der  centro = solo el círculo de 40 cm
# BOTÓN B: detener. Con el robot quieto: rumbo en 0 (prueba a mano del giroscopio).
# EN REPOSO la pantalla muestra el rumbo del giroscopio: PRUEBA A MANO -> poner el robot contra
#   una escuadra, B (rumbo 0), girarlo A MANO una vuelta completa hasta la misma escuadra y leer
#   la pantalla. GYRO_SCALE = 360 / lectura (si marca ~277, GYRO_SCALE = 1.3).
#
# HOJA DE CALIBRACIÓN (anotar y pasar a software/core/config.py):
#   - avance:  real / pedido. Si avanza 19 en vez de 20 -> RUEDA_D * 19/20  ("sprocket_d")
#   - recto:   si se desvía siempre al mismo lado -> LR ("lr_ratio")
#   - giros:   real en el piso vs 90°. Si el giro real se pasa o se queda corto y la pantalla dice
#              90 -> el giroscopio: GYRO_SCALE ("gyro_scale"); si la pantalla tampoco dice 90 ->
#              ajustar KP_GIRO / GIRO_MIN
#   - círculo: diámetro real (medir el rastro o la marca) vs 40. Si sale más chico/grande ->
#              SKID ("skid": derrape de las orugas al girar)

import event, time, math, cyberpi, mbot2

# ---------------------------------------------------------------- medidas del robot
RUEDA_D = 4.18        # CALIBRADO (prueba de 1 m): diámetro efectivo de la oruga, cm
LR = 0.977            # CALIBRADO (1 m): cm por grado izquierda / derecha (63.5 / 65)
ENTRE_ORUGAS = 18.35  # MEDIDO: centro a centro de las orugas (A 15.35 + B 3.0)
SKID = 1.3            # TODO(calibrar con el círculo): derrape de las orugas al girar
GYRO_SCALE = 1.0      # TODO(calibrar con la prueba a mano): grados reales por grado leído

# ---------------------------------------------------------------- ajustes del control
KP_RECTO = 1.5        # rpm por grado de desvío al avanzar
KP_GIRO = 1.2         # rpm por grado que falta al girar
KP_CIRCULO = 2.0      # rpm por grado de desvío del círculo
GIRO_MIN = 12         # TODO(ajustar) rpm mínimas para que las orugas giren en el lugar
TOL_GIRO = 1.0        # grados
RAMPA_S = 0.4         # s de arranque suave
RPM = 50              # velocidad de las pruebas

CM_POR_GRADO = math.pi * RUEDA_D / 360.0
K_IZQ = CM_POR_GRADO * 2 * LR / (1 + LR)
K_DER = CM_POR_GRADO * 2 / (1 + LR)

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
    """Grados acumulados, + = a la DERECHA."""
    return -cyberpi.get_rotation('z') * GYRO_SCALE - rumbo_cero


def orugas_cm():
    """(izquierda, derecha) en cm, + = adelante."""
    return mbot2.EM_get_angle("EM1") * K_IZQ, -mbot2.EM_get_angle("EM2") * K_DER


def limitar(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def anotar(texto):
    """Una línea de resultados en la pantalla (consola del CyberPi)."""
    cyberpi.console.println(texto)


# ---------------------------------------------------------------- movimientos
def avanzar_cm(cm, rpm=RPM):
    """Avanza `cm` (negativo = atrás) por encoders sosteniendo el rumbo con el giroscopio."""
    h_obj = rumbo()
    signo = 1 if cm >= 0 else -1
    meta = abs(cm)
    l0, r0 = orugas_cm()
    inicio = time.time()
    while True:
        if detener_ejecucion:
            parar()
            return False
        l, r = orugas_cm()
        falta = meta - abs(((l - l0) + (r - r0)) / 2.0)
        if falta <= 0.3:
            break
        v = rpm
        t = time.time() - inicio
        if t < RAMPA_S:
            v = rpm * max(0.3, t / RAMPA_S)
        if falta < 6:
            v = max(15, rpm * falta / 6.0)
        corr = limitar(KP_RECTO * (h_obj - rumbo()), -rpm * 0.5, rpm * 0.5)
        motores(signo * v + corr, (signo * v - corr) * LR)
        if t > 2 + meta / 3.0:
            break
        time.sleep(0.01)
    parar()
    time.sleep(0.3)
    l, r = orugas_cm()
    anotar("avanza %d: izq %.1f der %.1f rumbo %+.1f" % (cm, l - l0, r - r0, rumbo() - h_obj))
    return True


def girar(angulo, rpm_max=40, timeout=6):
    """Gira `angulo` grados en el lugar (+ = derecha): frena al acercarse y corrige."""
    h0 = rumbo()
    meta = h0 + angulo
    quieto = 0
    inicio = time.time()
    ok = False
    while time.time() - inicio < timeout:
        if detener_ejecucion:
            parar()
            return False
        err = meta - rumbo()
        if abs(err) < TOL_GIRO:
            parar()
            quieto += 1
            if quieto >= 5:
                ok = True
                break
        else:
            quieto = 0
            v = limitar(KP_GIRO * abs(err), GIRO_MIN, rpm_max)
            if err > 0:
                motores(v, -v)
            else:
                motores(-v, v)
        time.sleep(0.01)
    parar()
    time.sleep(0.3)
    anotar("gira %+d: giro %+.1f (%.1f s)" % (angulo, rumbo() - h0, time.time() - inicio))
    return ok


def circulo(diametro=40.0, rpm=40, lado=-1):
    """Una vuelta completa sobre un círculo de `diametro` cm (centro del robot); lado -1 =
    hacia la izquierda, +1 = derecha. Velocidades de las orugas según el radio (con el derrape
    SKID) y el giroscopio corrige: el rumbo debe avanzar 360° en pi*diametro cm."""
    R = diametro / 2.0
    be = ENTRE_ORUGAS * SKID / 2.0               # medio ancho "efectivo" al girar
    adentro = (R - be) / R                       # proporción de la oruga de adentro
    afuera = (R + be) / R
    h0 = rumbo()
    l0, r0 = orugas_cm()
    largo = math.pi * diametro
    inicio = time.time()
    while True:
        if detener_ejecucion:
            parar()
            return False
        l, r = orugas_cm()
        s = ((l - l0) + (r - r0)) / 2.0           # recorrido del centro del robot
        if s >= largo:
            break
        h_obj = h0 + lado * s / R * 180.0 / math.pi
        corr = limitar(KP_CIRCULO * (h_obj - rumbo()), -rpm, rpm)
        v = rpm
        t = time.time() - inicio
        if t < RAMPA_S:
            v = rpm * max(0.3, t / RAMPA_S)
        if lado < 0:
            izq, der = v * adentro, v * afuera
        else:
            izq, der = v * afuera, v * adentro
        motores(izq + corr, (der - corr) * LR)
        if t > 5 + largo / 4.0:
            break
        time.sleep(0.01)
    parar()
    time.sleep(0.3)
    l, r = orugas_cm()
    anotar("circulo %d: izq %.0f der %.0f" % (diametro, l - l0, r - r0))
    anotar("  rumbo %+.0f (debe %+d)" % (rumbo() - h0, lado * 360))
    return True


# ---------------------------------------------------------------- pruebas
def secuencia():
    pasos = [lambda: avanzar_cm(20), lambda: girar(-90), lambda: avanzar_cm(10),
             lambda: girar(90), lambda: girar(-90), lambda: avanzar_cm(20),
             lambda: circulo(40, 40, -1)]
    for paso in pasos:
        if not paso():
            return False
        time.sleep(0.5)
    return True


def correr(nombre, accion):
    global ejecutar, detener_ejecucion
    if ejecutar:
        return
    ejecutar = True
    detener_ejecucion = False
    cyberpi.console.clear()
    anotar(nombre)
    time.sleep(0.5)
    ok = accion()
    parar()
    anotar("FIN" if ok else "DETENIDO / TIEMPO")
    ejecutar = False


@event.start
def on_start():
    cyberpi.console.clear()
    anotar("CALIBRATIONS")
    anotar("A secuencia  B stop/rumbo 0")
    anotar("joy: ^50cm v360 <90 >90 o circ")


@event.is_press('a')
def prueba_secuencia():
    correr("SECUENCIA", secuencia)


@event.is_press('up')
def prueba_lineal():
    correr("LINEAL 50 cm", lambda: avanzar_cm(50))


@event.is_press('down')
def prueba_360():
    correr("GIRO 360", lambda: girar(360, 40, 12))


@event.is_press('left')
def prueba_izq():
    correr("GIRO 90 IZQ", lambda: girar(-90))


@event.is_press('right')
def prueba_der():
    correr("GIRO 90 DER", lambda: girar(90))


@event.is_press('middle')
def prueba_circulo():
    correr("CIRCULO 40 cm", lambda: circulo(40, 40, -1))


@event.is_press('b')
def detener():
    global detener_ejecucion, rumbo_cero
    if ejecutar:
        detener_ejecucion = True
        parar()
    else:
        rumbo_cero = -cyberpi.get_rotation('z') * GYRO_SCALE
        cyberpi.console.clear()
        anotar("RUMBO = 0: girar a mano 360")


@event.start
def main():
    # en reposo: rumbo del giroscopio en grande (prueba a mano)
    while True:
        if not ejecutar:
            cyberpi.display.show_label("rumbo %d" % int(round(rumbo())), 16, "bottom_mid")
        time.sleep(0.25)
