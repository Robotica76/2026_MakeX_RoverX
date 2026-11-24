## @file pista.py
#  @brief MEDIDAS DE LA PISTA que usa el programa AUTÓNOMO, por nivel.
#  @details Aquí están TODAS las medidas de la pista de las que depende la ruta, separadas por nivel.
#  Cada nivel tiene dos partes:
#    - "Mediciones que necesita el robot": se miden con flexómetro en la pista real.
#    - "Ajustes de la maniobra": retrocesos, holguras y rumbos; se afinan probando en la pista.
#
#  Los valores de ahora salen del plano del reglamento (simulation/arena.py) y se probaron en el
#  simulador. Los marcados  # MEDIR  hay que confirmarlos en la pista; al medirlos, cambiar
#  "MEDIR" por "MEDIDO".
#
#  Después de cambiar algo:
#    1) python3 -m pytest              (que todo siga pasando)
#    2) python3 simulation/run_auto.py (ver la ruta en el simulador)
#    3) python3 tools/scripts/bundle_for_mblock.py --all   y volver a cargar mblock/rover_x_auto.py
#
#  Las medidas marcadas [MENU] también se pueden cambiar desde el menú de ajustes del CyberPi
#  (grupo "Pista"). OJO: si ya se guardaron desde el menú, en el robot manda lo guardado y no lo
#  que diga este archivo (para volver a este archivo: menú -> "Restaurar valores").
#
#  UNIDADES Y MAPA
#    - Distancias en cm, ángulos en grados.
#    - Posiciones (x, y) en el mapa de la vista superior del reglamento: x hacia la DERECHA desde
#      la orilla izquierda de la arena, y hacia ABAJO desde la orilla de arriba.
#    - Rumbos: 0 = derecha (este), 90 = abajo (sur), 180 = izquierda (oeste), -90 = arriba (norte).
#    - "Centro de giro" = punto medio de las orugas; las sondas de línea van 5.2 cm delante de él.
#
#  Medidas del robot que importan para las holguras: 21.35 cm de ancho x 20.3 de largo; girando en
#  el lugar las esquinas barren 16 cm (nariz) y 13.6 cm (cola).


# =====================================================================================
# ---------------------------------------- NIVEL 1 (zona roja)
# Misiones 1 a 3: seguir la línea hasta el cruce del cubo, tomar el cubo, subir la rampa gris.
# Se recorre POR COMANDOS (avanzar X cm con el rumbo sostenido por el giroscopio).
# =====================================================================================

# ---- Mediciones que necesita el robot
nivel_1_inicio_x = 12.6                  # MEDIR  x del centro de giro al colocar el robot en la
nivel_1_inicio_y = 12.9                  # MEDIR  zona roja (sobre la X, mirando al sur)
nivel_1_inicio_a_linea_inferior_cm = 33.8  # MEDIR  de la zona roja (centro de giro) al cruce con
                                         # la línea de abajo, yendo al sur
nivel_1_linea_inferior_y = 43.2          # MEDIR  y del centro de giro cuando las sondas VEN la
                                         # línea de abajo (ahí se gira; la pared está en y 61)
nivel_1_cruce_a_cruce_del_cubo_cm = 59.0  # MEDIR  del cruce de abajo hasta tener las sondas sobre
                                         # el cruce frente al cubo, yendo al este
nivel_1_cruce_a_pedestal_cm = 12.4       # MEDIR [MENU] de donde las sondas ven el cruce (su orilla
                                         # cercana) al CENTRO del pedestal del cubo
nivel_1_linea_del_cubo_y = 49.0          # MEDIR  y de la línea del cruce frente al cubo
nivel_1_cubo_a_linea_de_rampa_cm = 18.4  # MEDIR  del cruce frente al cubo, al norte, hasta la altura
                                         # de la línea que sube a la rampa
nivel_1_linea_a_pie_de_rampa_cm = 18.7   # MEDIR  de ahí, al este, al pie de la rampa gris

# ---- Ajustes de la maniobra
nivel_1_x_de_bajada = 15.4               # x por la que baja: 2.8 cm a la derecha de la línea, porque
                                         # la cola barre 13.6 al girar y la orilla está en x 0
nivel_1_y_hacia_cubo = 47.4              # y de la recta hacia el cubo (pedestal en y 45.6..51.8): así
                                         # el pivote de después pasa a ~1.5 cm de la pared
nivel_1_holgura_cruce_cm = 6.0           # cada tramo se queda corto esto y termina buscando el cruce
nivel_1_retroceso_antes_de_girar_cm = 1.5  # el robot se pasa ~1 cm al frenar sobre la línea
nivel_1_retroceso_para_pivotear_cm = 11.0  # tras el cubo, para pivotear al norte sin barrer el pedestal
nivel_1_retroceso_tras_cubo_cm = 4.5     # el retroceso que ya hizo la misión 2 (lo descuenta)


# =====================================================================================
# ---------------------------------------- NIVEL 2 (zona amarilla)
# Misión 4 (dejar el cubo en el pedestal), 5 (cruzar la zona de obstáculos en diagonal, sin
# línea) y 6 (subir la rampa blanca al nivel 3).
# =====================================================================================

# ---- Mediciones que necesita el robot
nivel_2_t_a_pedestal_cm = 8.8            # MEDIR  línea seguida desde la T hasta dejar la garra sobre
                                         # el centro del pedestal (con 9.6 la nariz quedaba a 0.3 cm)
nivel_2_colocado_x = 131.9               # MEDIR  centro de giro al colocarlo en la T, retroceder y
nivel_2_colocado_y = 30.4                # MEDIR  girar al sur (arranque o reinicio en el nivel 2)
nivel_2_tras_dejar_cubo_x = 133.9        # MEDIR  centro de giro al terminar la misión 4
nivel_2_tras_dejar_cubo_y = 26.8         # MEDIR  (mirando al sur)
nivel_2_giro_al_pasillo_x = 134.2        # MEDIR  punto donde gira al este para entrar al pasillo:
nivel_2_giro_al_pasillo_y = 43.5         # MEDIR  y <= 61 - 16 - 1.5 (la nariz barre 16 cm)
nivel_2_abertura_izquierda_y = 45.0      # MEDIR  centro de la abertura del muro izquierdo (y 30.5..60.2)
nivel_2_abertura_derecha_y = 15.9        # MEDIR  centro de la abertura del muro derecho (y 0.9..30.5)
nivel_2_diagonal_punto_x = 212.7         # MEDIR  la diagonal pasa por el centro de la abertura derecha
nivel_2_diagonal_punto_y = 16.0          # MEDIR
nivel_2_diagonal_deteccion_x = 208.96    # MEDIR  centro de giro cuando la sonda L2 toca la línea de
nivel_2_diagonal_deteccion_y = 17.82     # MEDIR  salida yendo por la diagonal
nivel_2_x_al_ver_diagonal = 218.6        # MEDIR  x del centro de giro cuando las sondas centrales ven
                                         # la diagonal de la línea yendo al este por la abertura derecha
nivel_2_rampa_centro_x = 228.15          # MEDIR  centro de la rampa blanca (x 216.7..239.6)
nivel_2_rampa_linea_x = 228.6            # MEDIR  la línea que baja por ella (0.45 cm al este del centro)

# ---- Ajustes de la maniobra
nivel_2_retroceso_en_t_cm = 3.0          # retroceso en la T antes de girar al sur (la nariz barre 16
                                         # y el muro del pasillo está a ~17)
nivel_2_retroceso_tras_dejar_cubo_cm = 6.0
nivel_2_diagonal_rumbo = -26.0           # rumbo de la diagonal (noreste): una sola recta por el centro
                                         # de las dos aberturas, 2.3 cm a las puntas de los muros
nivel_2_diagonal_inicio_x = 150.5        # x del centro de giro al pivotear para quedar sobre la diagonal
nivel_2_diagonal_max_cm = 110.0          # tope por encoders (sobre los recortes patina: miden de más)
nivel_2_diagonal_vel = 14                # % potencia: lento y con el rumbo firme sobre los recortes
nivel_2_diagonal_kp = 2.5                # firmeza del rumbo; si oscila en el robot real, bajarlo
nivel_2_x_giro_salida = 223.0            # girar cuando la cola rebasa la punta del muro (x 212.7)
nivel_2_rumbo_hacia_rampa = 60           # al salir del pasillo: las sondas quedan junto a la línea
nivel_2_avance_hacia_rampa_cm = 5.5      # en diagonal hasta x ~226.6 (esquina a ~1.3 cm de la orilla)
nivel_2_rampa_retroceso_cm = 8.0         # la nariz ya estaba sobre el pie de la rampa: centrarse en plano


# =====================================================================================
# ---------------------------------------- NIVEL 3 (zona azul)
# Misión 6 (termina de entrar al nivel 3), 7 (mover el prisma a la zona punteada) y 9 (ir a la
# escalera). La misión 8 (pelota) está omitida.
# =====================================================================================

# ---- Mediciones que necesita el robot
nivel_3_zona_azul_a_dentro_cm = 19.2     # MEDIR  avance desde que las sondas ven la zona azul hasta
                                         # quedar completamente dentro (antes de la curva en S)
nivel_3_orilla_zona_azul_x = 228.8       # MEDIR  centro de giro al salir de la zona azul (sondas en
nivel_3_orilla_zona_azul_y = 70.8        # MEDIR  la orilla de la zona)
nivel_3_orilla_a_prisma_cm = 3.6         # MEDIR  de la orilla de la zona azul a la altura del prisma
nivel_3_rumbo_a_zona_punteada = 107      # MEDIR  rumbo del prisma hacia la zona punteada
nivel_3_prisma_a_zona_punteada_cm = 22.0  # MEDIR  avance con el prisma hasta dejarlo en la zona
nivel_3_frente_escalera_x = 166.1        # MEDIR  punto al sur del tramo recto de la línea, frente a
nivel_3_frente_escalera_y = 96.4         # MEDIR  la escalera

# ---- Ajustes de la maniobra
nivel_3_retroceso_antes_de_bajar_garra_cm = 4.0  # para bajar la garra sin tocar el prisma
nivel_3_avance_para_tomar_prisma_cm = 6.5  # al oeste hasta tener el prisma entre los dedos
nivel_3_retroceso_tras_soltar_cm = 8.0
nivel_3_retroceso_antes_de_escalera_cm = 5.0  # para que las sondas busquen la línea hacia adelante

# ---- Pelota (misión 8): OMITIDA por ahora; se guardan las medidas para después
nivel_3_pelota_linea_a_pedestal_cm = 23.6  # MEDIR [MENU] del centro de la línea (en la marca roja)
                                         # al centro del pedestal de la pelota
nivel_3_pelota_rumbo = 113               # MEDIR  rumbo hacia el pedestal de la pelota
nivel_3_pelota_rumbo_de_regreso = -152   # MEDIR  rumbo de la línea en la segunda marca


# =====================================================================================
# ---------------------------------------- NIVEL 4 (zona verde)
# Misión 9 (termina de subir la escalera), 10 (rodear la torre), 11 (leer las tarjetas de
# colores) y 13 (meta azul). La misión 12 (pelota en la casilla roja) está omitida.
# =====================================================================================

# ---- Mediciones que necesita el robot
nivel_4_escalera_a_dentro_cm = 8.0       # MEDIR  avance desde que las sondas ven la zona verde (en la
                                         # cima de la escalera); entre escalera y torre hay ~20 cm
nivel_4_torre_lado_cm = 10.0             # MEDIR [MENU] lado de la torre
nivel_4_zona_verde_a_torre_cm = 7.5      # MEDIR [MENU] del borde de salida de la zona verde a la
                                         # cara de la torre
nivel_4_pasar_torre_al_oeste_cm = 37.0   # MEDIR  avance al oeste, paralelo a la línea, hasta pasar
                                         # la torre (vuelve a la línea cerca de la 1a tarjeta)
nivel_4_retroceso_para_tarjetas_cm = 7.0  # MEDIR  la tarjeta más cercana a la torre queda detrás de
                                         # las sondas; la cola queda a ~1.5 cm de la torre

# ---- Ajustes de la maniobra
nivel_4_holgura_torre_cm = 2.0           # [MENU] holgura al rodear (con 4 cm volvía sobre una tarjeta)
nivel_4_rodeo_rumbo = 110                # sur-suroeste: lejos de la orilla del nivel 4
nivel_4_rodeo_cm = 20.0                  # al girar al oeste queda a ~2 cm de la esquina de la torre
                                         # (con 17 la tocaba)
# PENDIENTE (simulador): entre la cara sur de la torre (y 96.5) y la orilla de la arena (y 122)
# hay 25.5 cm y el robot mide 21.35: el rodeo pasa muy justo; hay que rediseñarlo.

# ---- Pelota en la barra (misión 12): OMITIDA por ahora
nivel_4_barra_lado = 1                   # [MENU] -1 = barra de pelotas a la izquierda de la línea, 1 = derecha
nivel_4_barra_avance_cm = 8.2            # MEDIR [MENU] avance desde la línea para dejar la garra
                                         # sobre el agujero
