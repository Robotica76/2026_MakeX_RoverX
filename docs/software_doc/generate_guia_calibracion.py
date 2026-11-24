#!/usr/bin/env python3
"""Genera la GUÍA DE CALIBRACIÓN para los alumnos (2 hojas): Guia_Calibracion.pdf.

    python3 docs/software_doc/generate_guia_calibracion.py        # .tex y .pdf

Hoja 1, EL ROBOT: dibujos del robot (vista superior y lateral) a escala, con cada variable
puesta sobre la parte del robot que describe y el número del paso de calibración que la ajusta;
los pasos de calibración y la tabla "si pasa esto, cambia esto".
Hoja 2, LA PISTA: la vista superior del reglamento con el recorrido del robot (simulador) y
cada medida de software/core/pista.py marcada con un número donde se mide; abajo, la lista
numerada con el valor actual y una casilla.

Todo se saca del programa: medidas del robot (cfg.P, geometry.py), medidas de la pista
(pista.py) y el recorrido (una corrida del simulador sin ruido). El script falla si:
  - CALIBRAR o SINTOMAS mencionan un parámetro que ya no existe,
  - hay una medida en pista.py que no está en MAPA (decir dónde va, o "none"),
  - la guía ya no cabe en 2 hojas.
"""
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import generate_software_doc as gsd  # noqa: E402

ROOT = gsd.ROOT
sys.path.insert(0, os.path.join(ROOT, "simulation"))
OUT_TEX = os.path.join(HERE, "Guia_Calibracion.tex")
MAP_IMG = "../../resources/reglamento/arena/arena_automatica_superior.jpg"   # relativo a HERE
MAX_PAGES = 2

# ------------------------------------------------------------------ hoja 1: calibrar el robot
# (número, qué, cómo, parámetros). Los {nombres} se revisan contra el programa. El número es el
# que aparece en los dibujos del robot junto a la variable.
CALIBRAR = [
    (1, "Signos", "tools/robot/api_check.py (Motores, IMU, Línea): avanzar = encoders +, "
     "girar a la derecha = rumbo +, nariz arriba = pitch +",
     "{LEFT_SIGN} {RIGHT_SIGN} {HEADING_SIGN} {PITCH_SIGN} (config.py)"),
    (2, "Distancia", "Menú > Probar recta 50 y medir con cinta", "{slip} = real / 50"),
    (3, "Orugas", "Prueba de 1 m de mBlock: lo que avanzó cada oruga",
     "{lr_ratio} = izquierda / derecha"),
    (4, "Giroscopio", "calibrations.py: B (rumbo 0), girar el robot A MANO una vuelta contra una "
     "escuadra y leer la pantalla", "{gyro_scale} = 360 / lectura"),
    (5, "Giros", "Menú > Probar giro 90 y medir con transportador",
     "se pasa: bajar {turn_kp}; no termina: subir {v_turn_min}"),
    (6, "Derrape", "calibrations.py, joystick al centro: círculo de 40 cm; medir el diámetro real",
     "{skid}"),
    (7, "Línea", "Robot centrado sobre la línea: Menú > Calibrar linea; revisar con Ver sensores "
     "(blanco ~0, negro ~1)", "se guarda solo; umbral {dark_th}"),
    (8, "Garra", "Menú > Garra: con < > cambiar y A para probar",
     "{claw_open} abierta, {claw_cube} cerrada en el cubo"),
    (9, "Brazo", "Con regla: altura y adelanto del centro de la garra con el servo a 0, 45 y 90",
     "{arm_h0} {arm_x0} {arm_h45} {arm_x45} {arm_h90} {arm_x90}"),
    (10, "Medidas del robot", "Con regla, desde el punto de referencia (naranja, sobre la caja)",
     "{ref_rot_fwd} {sensor_fwd} {robot_w} {belt_w} {track_c}"),
]

SINTOMAS = [
    ("Todas las distancias salen cortas o largas", "{slip} (paso 2)"),
    ("Dice 90° pero en el piso giró otra cosa", "{gyro_scale} (paso 4)"),
    ("Gira de más u oscila al terminar un giro", "bajar {turn_kp}; subir {turn_tol}"),
    ("Se tuerce al avanzar recto", "subir {head_kp}"),
    ("Zigzaguea siguiendo la línea", "bajar {kp}, subir {kd}"),
    ("Se sale de la línea en las curvas", "bajar {v_line}; subir {kp}"),
    ("Pierde la línea en zonas de color o con otra luz", "Calibrar linea (paso 7); {dark_th}"),
    ("Cuenta cruces de más (marcas verdes)", "subir {cross_th}"),
    ("No ve un cruce", "bajar {cross_th}; subir {cross_win}"),
    ("La garra queda antes o después del cubo", "{nivel_1_cruce_a_pedestal_cm} (hoja 2)"),
    ("La garra choca con el pedestal al bajar", "{h_pedestal}"),
    ("Suelta el cubo o no lo aprieta", "{claw_cube} (paso 8)"),
    ("Choca con una pared o un muro", "las medidas de ese nivel (hoja 2)"),
    ("Se queda sin fuerza o resbala en rampa/escalera", "subir {v_ramp} / {v_stairs}"),
    ("Se levanta de la nariz o se voltea al subir", "bajar {tip_slew}; {h_climb}"),
    ("Lee mal los colores", "{color_mode} = 1 y {COLOR_REF}"),
    ("Sigue con un valor viejo aunque cambié el archivo", "Menú > Restaurar valores y B"),
]

# ------------------------------------------------------------------ hoja 2: dónde va cada medida
# (variable de pista.py, tipo, argumentos):
#   "pt"            punto: variable sin _x/_y (dibuja un punto en (x, y))
#   "x", (y0, y1)   recta x = valor, dibujada entre y0 y y1 (cm del mapa)
#   "y", (x0, x1)   recta y = valor, dibujada entre x0 y x1
#   "dist", paso, rumbo   flecha de largo = valor desde donde EMPIEZA ese paso del simulador.
#                   paso = (misión, índice[, índice dentro de un if_...]); rumbo de mapa, None =
#                   hacia donde mira el robot, "back" = hacia atrás, o el nombre de otra variable
#   "dist_end", paso, rumbo   igual, pero la flecha TERMINA donde termina el paso
#   "head", paso    flecha punteada con el rumbo = valor desde donde empieza el paso
#   "none"[, nota]  no va en el mapa (solo en la lista, con la nota)
MAPA = [
    ("nivel_1_inicio", "pt"),
    ("nivel_1_inicio_a_linea_inferior_cm", "dist", (1, 4), 90),
    ("nivel_1_linea_inferior_y", "y", (2, 28)),
    ("nivel_1_cruce_a_cruce_del_cubo_cm", "dist", (1, 9), 0),
    ("nivel_1_cruce_a_pedestal_cm", "dist", (2, 0), None),
    ("nivel_1_linea_del_cubo_y", "y", (62, 92)),
    ("nivel_1_cubo_a_linea_de_rampa_cm", "dist", (3, 2), -90),
    ("nivel_1_linea_a_pie_de_rampa_cm", "dist", (3, 3), 0),
    ("nivel_1_x_de_bajada", "x", (16, 40)),
    ("nivel_1_y_hacia_cubo", "y", (22, 60)),
    ("nivel_1_holgura_cruce_cm", "none"),
    ("nivel_1_retroceso_antes_de_girar_cm", "dist", (1, 7), "back"),
    ("nivel_1_retroceso_para_pivotear_cm", "dist", (3, 0), "back"),
    ("nivel_1_retroceso_tras_cubo_cm", "dist", (2, 4), "back"),
    ("nivel_2_t_a_pedestal_cm", "dist", (4, 2), -90),
    ("nivel_2_colocado", "pt"),
    ("nivel_2_tras_dejar_cubo", "pt"),
    ("nivel_2_giro_al_pasillo", "pt"),
    ("nivel_2_abertura_izquierda_y", "y", (140, 166)),
    ("nivel_2_abertura_derecha_y", "y", (198, 224)),
    ("nivel_2_diagonal_punto", "pt"),
    ("nivel_2_diagonal_deteccion", "pt"),
    ("nivel_2_x_al_ver_diagonal", "x", (3, 26)),
    ("nivel_2_rampa_centro_x", "x", (30, 62)),
    ("nivel_2_rampa_linea_x", "none", "junto al de rampa_centro_x"),
    ("nivel_2_retroceso_en_t_cm", "dist", (5, 0, 1), "back"),
    ("nivel_2_retroceso_tras_dejar_cubo_cm", "dist", (4, 5), "back"),
    ("nivel_2_diagonal_rumbo", "head", (5, 5)),
    ("nivel_2_diagonal_inicio_x", "x", (34, 58)),
    ("nivel_2_diagonal_max_cm", "none"),
    ("nivel_2_diagonal_vel", "none"),
    ("nivel_2_diagonal_kp", "none"),
    ("nivel_2_x_giro_salida", "x", (3, 26)),
    ("nivel_2_rumbo_hacia_rampa", "head", (5, 10)),
    ("nivel_2_avance_hacia_rampa_cm", "dist", (5, 11), "nivel_2_rumbo_hacia_rampa"),
    ("nivel_2_rampa_retroceso_cm", "dist", (5, 14), "back"),
    ("nivel_3_zona_azul_a_dentro_cm", "dist_end", (6, 1), None),
    ("nivel_3_orilla_zona_azul", "pt"),
    ("nivel_3_orilla_a_prisma_cm", "dist", (7, 3), 90),
    ("nivel_3_rumbo_a_zona_punteada", "head", (7, 10)),
    ("nivel_3_prisma_a_zona_punteada_cm", "dist", (7, 11), "nivel_3_rumbo_a_zona_punteada"),
    ("nivel_3_frente_escalera", "pt"),
    ("nivel_3_retroceso_antes_de_bajar_garra_cm", "dist", (7, 6), "back"),
    ("nivel_3_avance_para_tomar_prisma_cm", "dist", (7, 9), 180),
    ("nivel_3_retroceso_tras_soltar_cm", "dist", (7, 13), "back"),
    ("nivel_3_retroceso_antes_de_escalera_cm", "dist", (9, 3), "back"),
    ("nivel_3_pelota_linea_a_pedestal_cm", "none", "misión de la pelota omitida"),
    ("nivel_3_pelota_rumbo", "none", "misión omitida"),
    ("nivel_3_pelota_rumbo_de_regreso", "none", "misión omitida"),
    ("nivel_4_escalera_a_dentro_cm", "dist_end", (9, -1), None),
    ("nivel_4_torre_lado_cm", "none", "no se usa ahora"),
    ("nivel_4_zona_verde_a_torre_cm", "none", "no se usa ahora"),
    ("nivel_4_pasar_torre_al_oeste_cm", "dist", (10, 3), 180),
    ("nivel_4_retroceso_para_tarjetas_cm", "dist", (11, 0), "back"),
    ("nivel_4_holgura_torre_cm", "none"),
    ("nivel_4_rodeo_rumbo", "head", (10, 2)),
    ("nivel_4_rodeo_cm", "dist", (10, 2), "nivel_4_rodeo_rumbo"),
    ("nivel_4_barra_lado", "none"),
    ("nivel_4_barra_avance_cm", "none", "misión 12 omitida"),
]
LEVEL_COLOR = {"1": "red!75!black", "2": "orange!85!black", "3": "blue!75!black",
               "4": "green!50!black"}


# ------------------------------------------------------------------ utilidades
def names_in(text):
    return re.findall(r"\{(\w+)\}", text)


def check_names(cfg, pista):
    bad = []
    for row in CALIBRAR + SINTOMAS:
        for n in names_in(" ".join(str(c) for c in row)):
            if n not in cfg.P and not hasattr(cfg, n) and not hasattr(pista, n):
                bad.append(n)
    if bad:
        sys.exit("La guía menciona parámetros que ya no existen: %s" % sorted(set(bad)))
    names = {n for n in dir(pista) if n.startswith("nivel_")}
    covered = set()
    for e in MAPA:
        if e[1] == "pt":
            covered |= {e[0] + "_x", e[0] + "_y"}
        else:
            covered.add(e[0])
    if names - covered:
        sys.exit("Medidas de pista.py sin lugar en MAPA (agregarlas): %s" % sorted(names - covered))
    if covered - names:
        sys.exit("MAPA menciona medidas que ya no están en pista.py: %s" % sorted(covered - names))


def fmt(text, tune):
    """Texto con {nombres} -> LaTeX; los parámetros del menú llevan una M."""
    out, pos = [], 0
    for m in re.finditer(r"\{(\w+)\}", text):
        out.append(gsd.tex(text[pos:m.start()]))
        n = m.group(1)
        out.append(gsd.tt(n) + (r"\menu" if n in tune else ""))
        pos = m.end()
    out.append(gsd.tex(text[pos:]))
    return "".join(out)


def var(n, tune):
    return gsd.tt(n) + (r"\menu" if n in tune else "")


# ------------------------------------------------------------------ corrida del simulador
def simulate():
    """Corre el autónomo completo en el simulador (sin ruido) y devuelve el recorrido y la pose
    del robot al empezar y al terminar cada paso. Marco del mapa: x este, y SUR, rumbo horario."""
    import physics
    import run_auto
    import autonomous
    import arena as ar
    holder = {}
    orig_init = physics.Sim.__init__

    def init(self, *a, **k):
        orig_init(self, *a, **k)
        holder["sim"] = self
    physics.Sim.__init__ = init
    poses = {}
    orig_step = autonomous.run_step

    def pose():
        s = holder["sim"]
        return (s.x, ar.H_CM - s.y, -s.yaw)

    def step(st):
        start = pose()
        ok = orig_step(st)
        poses.setdefault(id(st), (start, pose()))
        return ok
    autonomous.run_step = step
    try:
        res = run_auto.run_once(level=1, noise=0.0, seed=0, record=True)
    finally:
        autonomous.run_step = orig_step
        physics.Sim.__init__ = orig_init
    path, seg = [], []
    for f in res["frames"]:
        p = (f[1], ar.H_CM - f[2])
        if seg and math.hypot(p[0] - seg[-1][0], p[1] - seg[-1][1]) > 8:
            path.append(seg)
            seg = []
        if not seg or math.hypot(p[0] - seg[-1][0], p[1] - seg[-1][1]) > 1.0:
            seg.append(p)
    if seg:
        path.append(seg)
    return path, poses, ar


def step_at(missions, path):
    m = next(x for x in missions.MISSIONS if x["id"] == path[0])
    st = m["steps"][path[1]]
    if len(path) > 2:
        sub = st[1] if st[0] == "if_placed" else st[2]
        st = sub[path[2]]
    return st


# ------------------------------------------------------------------ hoja 1: dibujos del robot
def badge(n):
    return r"\bdg{%d}" % n


def robot_top(P, geo):
    """Vista superior a escala (1 cm del robot = 0.27 cm del papel). Adelante = arriba."""
    tw = geo.track_w()
    bw = P["belt_w"]
    rear, front = geo.rear_len(), geo.track_front()
    sen = geo.fwd("sensor_fwd")
    ref = -P["ref_rot_fwd"]
    claw = geo.claw_reach(P["h_carry"])
    lat = [geo.probe_lat(i, P["probe_pitch"], P["probe_outer"]) for i in range(4)]
    o = [r"\begin{tikzpicture}[x=0.27cm,y=0.27cm,>=Stealth,font=\scriptsize,"
         r"lb/.style={fill=white,inner sep=1pt,align=center}]"]
    for sx in (-1, 1):                                  # orugas
        x0 = sx * tw / 2 - bw / 2
        o.append(r"\draw[fill=gray!55,rounded corners=2pt] (%.2f,%.2f) rectangle (%.2f,%.2f);"
                 % (x0, -rear, x0 + bw, front))
        for k in range(int(rear + front) // 2):
            y = -rear + 1 + 2 * k
            o.append(r"\draw[gray!25] (%.2f,%.2f) -- (%.2f,%.2f);" % (x0, y, x0 + bw, y))
    o.append(r"\draw[fill=blue!10,rounded corners=3pt] (%.2f,%.2f) rectangle (%.2f,%.2f);"
             % (-tw / 2 + bw / 2, -rear + 1.5, tw / 2 - bw / 2, front - 2.5))
    o.append(r"\node[gray] at (0,%.2f) {mBot2 / CyberPi};" % (-rear + 2.2))
    # brazo y garra
    o.append(r"\draw[fill=gray!20] (%.2f,%.2f) rectangle (%.2f,%.2f);" % (-2.4, front - 3,
                                                                     2.4, claw - 1.5))
    o.append(r"\draw[very thick] (-2.4,%.2f) -- (-2.4,%.2f) (2.4,%.2f) -- (2.4,%.2f);"
             % (claw - 1.5, claw + 2, claw - 1.5, claw + 2))
    o.append(r"\node[lb,anchor=west] at (2.8,%.2f) {garra %s};" % (claw + 1.5, badge(8)))
    # barra del sensor de línea y sondas
    o.append(r"\draw[fill=black!70] (%.2f,%.2f) rectangle (%.2f,%.2f);" % (lat[0] - 0.8, sen - 0.4,
                                                                     lat[3] + 0.8, sen + 0.4))
    for x, n in zip(lat, ("L2", "L1", "R1", "R2")):
        o.append(r"\fill[yellow] (%.2f,%.2f) circle (0.32);" % (x, sen))
        o.append(r"\node[font=\tiny] at (%.2f,%.2f) {%s};" % (x, sen - 1.1, n))
    # centro de giro y referencia
    o.append(r"\fill[red] (0,0) circle (0.45); \draw[red] (-1.2,0) -- (1.2,0) (0,-1.2) -- (0,1.2);")
    o.append(r"\fill[orange] (0,%.2f) circle (0.45);" % ref)
    # cotas
    L = -tw / 2 - bw / 2
    R = tw / 2 + bw / 2
    o.append(r"\draw[<->] (%.2f,%.2f) -- node[lb] {%s %s} (%.2f,%.2f);"
             % (L, -rear - 2.5, var("robot_w", T), badge(10), R, -rear - 2.5))
    o.append(r"\draw[<->] (%.2f,%.2f) -- node[lb,below=1pt] {%s} (%.2f,%.2f);"
             % (L, -rear - 0.8, var("belt_w", T), L + bw, -rear - 0.8))
    o.append(r"\draw[<->,red] (1.6,%.2f) -- node[lb,right,font=\tiny] {%s} (1.6,0);"
             % (ref, var("ref_rot_fwd", T)))
    o.append(r"\draw[<->,blue!70!black] (%.2f,%.2f) -- node[lb,right] {%s} (%.2f,%.2f);"
             % (lat[3] + 3.6, ref, var("sensor_fwd", T), lat[3] + 3.6, sen))
    o.append(r"\draw[densely dotted] (0,%.2f) -- (%.2f,%.2f) (%.2f,%.2f) -- (%.2f,%.2f);"
             % (ref, lat[3] + 4.3, ref, lat[3] + 1, sen, lat[3] + 4.3, sen))
    o.append(r"\node[lb,anchor=east,text=red] at (%.2f,-1.6) {centro de giro};" % (L - 0.3))
    o.append(r"\node[lb,anchor=east,text=orange!80!black] at (%.2f,%.2f) {referencia};"
             % (L - 0.3, ref - 1.2))
    o.append(r"\node[lb,anchor=west] at (%.2f,%.2f) {sondas: Calibrar linea %s\\ %s};"
             % (R + 0.6, sen + 3.2, badge(7), var("dark_th", T)))
    # rumbo (giroscopio) y derrape
    o.append(r"\draw[->,thick,purple] (160:3.2) arc (160:95:3.2);")
    o.append(r"\node[lb,text=purple,anchor=east] at (%.2f,2.6) {rumbo + = derecha\\ %s %s};"
             % (L - 0.3, var("gyro_scale", T), badge(4)))
    o.append(r"\draw[->,dashed,purple!60] (0,0) ++(200:7) arc (200:250:7);")
    o.append(r"\node[lb,text=purple!80,anchor=north east] at (%.2f,-6.6) {giro: %s %s\\ "
             r"%s %s};" % (L - 0.3, var("turn_kp", T), badge(5), var("skid", T), badge(6)))
    # orugas: avance
    o.append(r"\node[lb,anchor=east] at (%.2f,%.2f) {oruga izq.\\ %s %s};"
             % (L - 0.3, front - 4, var("lr_ratio", T), badge(3)))
    o.append(r"\node[lb,anchor=west] at (%.2f,%.2f) {oruga der.\\ %s %s};"
             % (R + 0.3, -rear + 3, var("slip", T), badge(2)))
    o.append(r"\draw[->,very thick,gray] (0,%.2f) -- (0,%.2f) node[above,black] {adelante};"
             % (claw + 2.5, claw + 5))
    o.append(r"\end{tikzpicture}")
    return "\n".join(o)


def robot_side(P, geo):
    """Vista lateral a escala: orugas, brazo a 0/45/90 y las alturas de la garra."""
    wr = P["wheel_d"] / 2
    c = geo.contact_half()
    nx, nh = geo.nose_fwd(), geo.nose_h()
    tf = geo.track_front()
    o = [r"\begin{tikzpicture}[x=0.21cm,y=0.21cm,>=Stealth,font=\scriptsize,"
         r"lb/.style={fill=white,inner sep=1pt,align=center}]"]
    o.append(r"\draw[thick,brown!60!black] (%.2f,0) -- (%.2f,0);" % (-c - 6, tf + 18))
    o.append(r"\draw[fill=gray!55,rounded corners=3pt] (%.2f,0) -- (%.2f,0) -- (%.2f,%.2f) -- "
             r"(%.2f,%.2f) -- (%.2f,%.2f) -- cycle;" % (-c - wr, c, nx + wr, nh + wr,
                                                       nx, nh + 2 * wr, -c - wr, 2 * wr))
    for x, y in ((-c, wr), (c, wr), (nx, nh + wr)):
        o.append(r"\draw[fill=white] (%.2f,%.2f) circle (%.2f);" % (x, y, wr * 0.8))
    o.append(r"\draw[fill=blue!10,rounded corners=2pt] (%.2f,%.2f) rectangle (%.2f,%.2f);"
             % (-c - 1, 2 * wr, c + 1, 2 * wr + 5))
    piv = (P["arm_pivot_fwd"] - P["ref_rot_fwd"], P["arm_pivot_h"])
    o.append(r"\draw[fill=gray!30] (%.2f,%.2f) rectangle (%.2f,%.2f);"
             % (piv[0] - 1, 2 * wr + 5, piv[0] + 0.6, piv[1] + 0.8))
    colors = {0: "red!70!black", 45: "orange!80!black", 90: "green!45!black"}
    for a in (0, 45, 90):
        h, x = geo.arm_at(a)
        cx = tf + x
        o.append(r"\draw[very thick,%s] (%.2f,%.2f) -- (%.2f,%.2f);" % (colors[a], piv[0], piv[1],
                                                                      cx, h))
        o.append(r"\draw[fill=%s] (%.2f,%.2f) circle (0.55);" % (colors[a], cx, h))
    # cotas del brazo a 0 grados y etiquetas de las tres posiciones
    h0, x0 = geo.arm_at(0)
    o.append(r"\draw[densely dotted] (%.2f,0) -- (%.2f,%.2f);" % (tf, tf, h0 + 2))
    o.append(r"\draw[<->,red!70!black] (%.2f,0) -- node[lb,right] {%s} (%.2f,%.2f);"
             % (tf + x0 + 1.6, var("arm_h0", T), tf + x0 + 1.6, h0))
    o.append(r"\draw[<->,red!70!black] (%.2f,%.2f) -- node[lb,below] {%s} (%.2f,%.2f);"
             % (tf, h0 - 2.2, var("arm_x0", T), tf + x0, h0 - 2.2))
    h45, x45 = geo.arm_at(45)
    h90, x90 = geo.arm_at(90)
    o.append(r"\node[lb,anchor=west,text=orange!80!black] at (%.2f,%.2f) {servo 45: %s, %s};"
             % (tf + x45 + 1, h45 + 0.6, var("arm_h45", T), var("arm_x45", T)))
    o.append(r"\node[lb,anchor=west,text=green!45!black] at (%.2f,%.2f) {servo 90: %s, %s};"
             % (tf + x90 + 1.2, h90 + 1, var("arm_h90", T), var("arm_x90", T)))
    o.append(r"\node[lb,anchor=west] at (%.2f,%.2f) {brazo %s (alturas del centro de la garra)};"
             % (-c - 6, h90 + 4.2, badge(9)))
    # alturas de trabajo
    for k, txt in (("h_pedestal", "sobre el pedestal"), ("h_carry", "llevar")):
        o.append(r"\draw[dashed,gray] (%.2f,%.2f) -- (%.2f,%.2f) node[lb,right,text=black] {%s %s};"
                 % (tf + 4, P[k], tf + 9, P[k], var(k, T), txt))
    o.append(r"\draw[fill=brown!30] (%.2f,0) rectangle (%.2f,10);" % (tf + 14, tf + 17))
    o.append(r"\node[font=\tiny,rotate=90] at (%.2f,5) {pedestal 10 cm};" % (tf + 15.5))
    # cabeceo
    o.append(r"\draw[->,thick,purple] (%.2f,%.2f) arc (-10:40:3);" % (-c - 1, 2 * wr + 7.5))
    o.append(r"\node[lb,text=purple,anchor=east] at (%.2f,%.2f) {nariz arriba =\\ pitch + %s};"
             % (-c - 1.6, 2 * wr + 8, badge(1)))
    o.append(r"\node[lb,anchor=north] at (%.2f,-0.6) {%s (del eje blanco a la 1a negra)};"
             % (0, var("track_c", T)))
    o.append(r"\end{tikzpicture}")
    return "\n".join(o)


def claw_view(P):
    """Garra vista desde arriba: abierta y cerrada en el cubo."""
    o = [r"\begin{tikzpicture}[x=0.21cm,y=0.21cm,font=\scriptsize,"
         r"lb/.style={fill=white,inner sep=1pt,align=center}]"]
    w = P["claw_open_w"]
    for dx, open_, lab in ((0, True, var("claw_open", T) + r"\\abierta"),
                           (11, False, var("claw_cube", T) + r"\\cerrada")):
        g = w / 2 if open_ else 2.5
        o.append(r"\draw[fill=gray!30] (%.2f,0) rectangle (%.2f,1);" % (dx - w / 2 - 0.6, dx + w / 2 + 0.6))
        for s in (-1, 1):
            o.append(r"\draw[line width=2pt] (%.2f,1) -- (%.2f,4.5);" % (dx + s * (w / 2 + 0.3),
                                                                        dx + s * g))
        if not open_:
            o.append(r"\draw[fill=yellow!40] (%.2f,1.8) rectangle (%.2f,6.8);" % (dx - 2.4, dx + 2.4))
        o.append(r"\node[lb] at (%.2f,-1.6) {%s};" % (dx, lab))
    o.append(r"\node[lb,anchor=west] at (16,3) {%s};" % badge(8))
    o.append(r"\end{tikzpicture}")
    return "\n".join(o)


T = set()       # parámetros del menú (se llena en build)


# ------------------------------------------------------------------ hoja 2: el mapa
def map_tikz(path, poses, ar, pista, ms, numbered):
    k = 26.0 / 1093.0                       # cm del papel por pixel de la imagen

    def P(x, ys):                           # mapa (cm, y sur) -> papel (cm)
        return ((20 + x / ar.S) * k, (570 - (25 + ys / ar.S)) * k)

    o = [r"\begin{tikzpicture}[>=Stealth,font=\scriptsize]",
         r"\node[anchor=south west,inner sep=0] at (0,0) "
         r"{\includegraphics[width=26cm]{%s}};" % MAP_IMG]
    for seg in path:                        # recorrido del robot en el simulador
        if len(seg) > 1:
            o.append(r"\draw[orange!90!black,line width=0.9pt] " + " -- ".join(
                "(%.2f,%.2f)" % P(*p) for p in seg) + ";")

    def mark(n, x, y, col, pos="above"):
        o.append(r"\node[circle,draw=%s,fill=white,text=%s,inner sep=0.6pt,minimum size=9pt,"
                 r"font=\tiny\bfseries,line width=0.6pt] at (%.2f,%.2f) {%d};" % (col, col, x, y, n))

    for n, e in numbered:
        name, kind = e[0], e[1]
        col = LEVEL_COLOR[name.split("_")[1]]
        if kind == "pt":
            x, y = P(getattr(pista, name + "_x"), getattr(pista, name + "_y"))
            o.append(r"\fill[%s] (%.2f,%.2f) circle (2pt);" % (col, x, y))
            mark(n, x + 0.28, y + 0.28, col)
        elif kind in ("x", "y"):
            v = getattr(pista, name)
            a, b = e[2]
            p0, p1 = (P(v, a), P(v, b)) if kind == "x" else (P(a, v), P(b, v))
            o.append(r"\draw[%s,densely dashed,line width=0.8pt] (%.2f,%.2f) -- (%.2f,%.2f);"
                     % (col, p0[0], p0[1], p1[0], p1[1]))
            end = p0 if kind == "x" else p1
            mark(n, end[0] + (0 if kind == "x" else 0.3), end[1] + (0.3 if kind == "x" else 0), col)
        elif kind in ("dist", "dist_end", "head"):
            st = step_at(ms, e[2])
            if id(st) not in poses:
                print("aviso: el simulador no corrió el paso de", name)
                continue
            start, end = poses[id(st)]
            if kind == "head":
                h, L, x0, y0 = getattr(pista, name), 14.0, start[0], start[1]
                style = "->,densely dotted,line width=0.9pt"
            else:
                hd = e[3]
                L = getattr(pista, name)
                ref = end if kind == "dist_end" else start
                if hd is None:
                    h = ref[2]
                elif hd == "back":
                    h = start[2] + 180
                elif isinstance(hd, str):
                    h = getattr(pista, hd)
                else:
                    h = hd
                x0, y0 = ref[0], ref[1]
                if kind == "dist_end":
                    x0 -= L * math.cos(math.radians(h))
                    y0 -= L * math.sin(math.radians(h))
                style = "->,line width=1.1pt"
            x1 = x0 + L * math.cos(math.radians(h))
            y1 = y0 + L * math.sin(math.radians(h))
            a, b = P(x0, y0), P(x1, y1)
            o.append(r"\draw[%s,%s] (%.2f,%.2f) -- (%.2f,%.2f);" % (col, style, a[0], a[1], b[0], b[1]))
            mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
            nx, ny = -(b[1] - a[1]), b[0] - a[0]          # a un lado de la flecha
            d = math.hypot(nx, ny) or 1
            mark(n, mx + 0.3 * nx / d, my + 0.3 * ny / d, col)
    o.append(r"\end{tikzpicture}")
    return "\n".join(o)


# ------------------------------------------------------------------ documento
def build():
    import run_auto  # noqa: F401  (rutas de los módulos y del simulador)
    path, poses, ar = simulate()
    cfg, ms = gsd.load_program()
    import pista
    import geometry as geo
    check_names(cfg, pista)
    T.clear()
    T.update(t[1] for t in cfg.TUNE)
    P = cfg.P
    lines = []
    w = lines.append
    w(r"""% ARCHIVO GENERADO por generate_guia_calibracion.py -- no editar a mano.
\documentclass[9pt]{extarticle}
\usepackage[utf8]{inputenc}
\usepackage[T1]{fontenc}
\usepackage{lmodern}
\usepackage{textcomp}
\usepackage[spanish,es-noshorthands]{babel}
\usepackage[landscape,letterpaper,margin=0.8cm,top=0.7cm,bottom=0.6cm]{geometry}
\usepackage{graphicx}
\usepackage{multicol}
\usepackage{array}
\usepackage{xcolor}
\usepackage{enumitem}
\usepackage{amssymb}
\usepackage{tikz}
\usetikzlibrary{arrows.meta}
\setlength{\parindent}{0pt}
\setlength{\columnsep}{0.5cm}
\pagestyle{empty}
\newcommand{\menu}{\textsuperscript{\tiny\sffamily\bfseries M}}
\newcommand{\bdg}[1]{\tikz[baseline=-0.6ex]\node[circle,fill=black,text=white,inner sep=0.5pt,
  minimum size=9pt,font=\tiny\bfseries]{#1};}
\newcommand{\hd}[1]{{\color{blue!60!black}\large\bfseries #1}\par\smallskip}
\begin{document}""")
    # ---------------- hoja 1
    w(r"{\LARGE\bfseries Rover X -- Calibración: el robot}\hfill{\small Los números "
      r"\bdg{1}\dots\bdg{10} son los pasos de calibración; \menu{} = se cambia en el menú del "
      r"CyberPi (autónomo, centro del joystick; B guarda).}\par\vspace{1pt}\hrule\vspace{4pt}")
    w(r"\begin{minipage}[t]{0.56\linewidth}\vspace{0pt}")
    w(r"\hd{Vista de arriba}")
    w(r"\centering " + robot_top(P, geo) + r"\par\medskip")
    w(r"\raggedright\hd{Vista de lado}")
    w(r"\centering " + robot_side(P, geo) + r"\hspace{0.3cm}" + claw_view(P))
    w(r"\end{minipage}\hfill")
    w(r"\begin{minipage}[t]{0.42\linewidth}\vspace{0pt}")
    w(r"\hd{Pasos de calibración}")
    w(r"{\footnotesize Una vez, y otra si se cambia algo del robot. Cambiar \textbf{una cosa a la "
      r"vez} y anotarla.}\par\smallskip")
    w(r"\footnotesize\begin{tabular}{@{}c>{\raggedright\arraybackslash}p{0.92\linewidth}@{}}")
    for n, what, how, change in CALIBRAR:
        w(r"\bdg{%d} & \textbf{%s.} %s.\newline $\Rightarrow$ %s \\[2pt]"
          % (n, gsd.tex(what), gsd.tex(how), fmt(change, T)))
    w(r"\end{tabular}\par\medskip")
    w(r"\hd{Si pasa esto, cambia esto}")
    w(r"\scriptsize\begin{tabular}{@{}>{\raggedright\arraybackslash}p{0.5\linewidth}"
      r">{\raggedright\arraybackslash}p{0.47\linewidth}@{}}")
    for sym, change in SINTOMAS:
        w(r"%s & %s \\" % (gsd.tex(sym), fmt(change, T)))
    w(r"\end{tabular}")
    w(r"\end{minipage}")
    w(r"\par\vfill{\scriptsize Después de cambiar \texttt{config.py} o \texttt{pista.py}: "
      r"\texttt{python3 -m pytest}, probar en el simulador, \texttt{python3 tools/scripts/"
      r"bundle\_for\_mblock.py -{}-all} y volver a cargar \texttt{mblock/rover\_x\_auto.py}. "
      r"Lo guardado desde el menú manda sobre los archivos.}")
    w(r"\newpage")
    # ---------------- hoja 2
    # solo las MEDICIONES (MEDIR / MEDIDO) llevan número y van en el mapa; los ajustes de la
    # maniobra se listan aparte (se afinan probando, no se miden)
    state = {}
    for _, _, subs in gsd.pista_sections(os.path.join(ROOT, "software/core/pista.py")):
        for _, rows in subs:
            for nm, _, st, _ in rows:
                state[nm] = st
    measured, tuning = [], []
    for e in MAPA:
        key = e[0] + "_x" if e[1] == "pt" else e[0]
        (measured if "MEDI" in state.get(key, "") else tuning).append(e)
    numbered = list(enumerate(measured, 1))
    drawable = [(n, e) for n, e in numbered if e[1] != "none"]
    w(r"{\LARGE\bfseries Rover X -- Calibración: la pista}\hfill{\small Cada número marca dónde se "
      r"mide esa variable de \texttt{software/core/pista.py}. Línea naranja = recorrido del "
      r"robot (simulador).}\par\vspace{1pt}\hrule\vspace{3pt}")
    w(r"\centering " + map_tikz(path, poses, ar, pista, ms, drawable) + r"\par\vspace{2pt}")
    w(r"\raggedright{\scriptsize Medir con flexómetro, en cm. \textbf{x} hacia la derecha desde la "
      r"orilla izquierda, \textbf{y} hacia abajo desde la orilla de arriba; rumbo 0 = derecha, "
      r"90 = abajo. Las posiciones son del \textbf{centro de giro} (punto rojo de la hoja 1). "
      r"$\bullet$ punto (x, y) \quad - - - recta x o y \quad $\rightarrow$ distancia \quad "
      r"$\cdots\!\!>$ rumbo. Escribir el valor medido en \texttt{pista.py} y cambiar MEDIR por "
      r"MEDIDO.}\par\vspace{2pt}")

    def value(e):
        name = e[0]
        if e[1] == "pt":
            return name + "_x/y", "%s, %s" % (getattr(pista, name + "_x"), getattr(pista, name + "_y"))
        return name, str(getattr(pista, name))

    by_level = {}
    for n, e in numbered:
        by_level.setdefault(e[0].split("_")[1], []).append((n, e))
    titles = {t.split()[1]: t for t, _, _ in gsd.pista_sections(os.path.join(ROOT, "software/core/pista.py"))}
    for lvl in sorted(by_level):
        w(r"\begin{minipage}[t]{0.245\linewidth}\tiny")
        w(r"{\small\bfseries\color{%s} %s}\par" % (LEVEL_COLOR[lvl], gsd.tex(titles.get(lvl, "NIVEL " + lvl))))
        w(r"\begin{tabular}{@{}r@{\,}>{\raggedright\arraybackslash}p{0.66\linewidth}r@{\,}l@{}}")
        for n, e in by_level[lvl]:
            short, v = value(e)
            short = short.replace("nivel_%s_" % lvl, "")
            note = ""
            if e[1] == "none":
                note = r" \textcolor{gray}{(%s)}" % gsd.tex(e[2] if len(e) > 2 else "sin mapa")
            key = e[0] + "_x" if e[1] == "pt" else e[0]
            done = r"$\checkmark$" if state.get(key, "").startswith("MEDIDO") else r"$\square$"
            w(r"\textbf{%d} & %s%s & %s & %s \\" % (n, gsd.tt(short), note, gsd.tex(v), done))
        w(r"\end{tabular}\end{minipage}\hfill")
    w(r"\par\vspace{2pt}{\scriptsize\textbf{Ajustes de la maniobra} (%d en \texttt{pista.py}: "
      r"retrocesos, holguras y rumbos como \texttt{retroceso\_para\_pivotear\_cm}): no se miden, se "
      r"afinan probando en la pista. Los nombres de la lista van con su nivel delante: "
      r"\texttt{inicio\_x/y} del nivel 1 = \texttt{nivel\_1\_inicio\_x} y "
      r"\texttt{nivel\_1\_inicio\_y}.}" % len(tuning))
    w(r"\end{document}")
    return "\n".join(lines)


def main():
    with open(OUT_TEX, "w", encoding="utf-8") as f:
        f.write(build())
    print("Generado", os.path.relpath(OUT_TEX, ROOT))
    pdflatex = gsd.find_tex("pdflatex")
    if not pdflatex:
        sys.exit("Falta pdflatex (MacTeX / TeX Live)")
    for _ in range(2):
        r = subprocess.run([pdflatex, "-interaction=nonstopmode", "-halt-on-error",
                            "Guia_Calibracion.tex"], cwd=HERE, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT)
        if r.returncode != 0:
            sys.exit("pdflatex falló:\n" + "\n".join(
                r.stdout.decode("utf-8", "replace").splitlines()[-30:]))
    log = open(os.path.join(HERE, "Guia_Calibracion.log"), encoding="latin-1").read()
    m = re.search(r"Output written on .*\((\d+) page", log)
    n = int(m.group(1)) if m else 0
    for ext in ("aux", "log", "out"):
        try:
            os.remove(os.path.join(HERE, "Guia_Calibracion." + ext))
        except OSError:
            pass
    print("PDF: %s (%d hojas)" % (os.path.relpath(os.path.join(HERE, "Guia_Calibracion.pdf"),
                                                  ROOT), n))
    if n > MAX_PAGES:
        sys.exit("La guía debe caber en %d hojas: acortar los textos" % MAX_PAGES)


if __name__ == "__main__":
    main()
