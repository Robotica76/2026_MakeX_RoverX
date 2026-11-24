#!/usr/bin/env python3
"""Genera la GUÍA RÁPIDA DE CALIBRACIÓN (una hoja, para los alumnos): Guia_Calibracion.pdf.

    python3 docs/software_doc/generate_guia_calibracion.py        # .tex y .pdf

Tres partes: 1) calibrar el robot, 2) medir la pista (lista sacada de software/core/pista.py:
siempre al día), 3) síntoma -> qué cambiar. Los textos de las partes 1 y 3 están aquí abajo
(CALIBRAR y SINTOMAS); el script revisa que cada parámetro que mencionan exista en el programa
(cfg.P, constantes de config.py o pista.py), así la hoja no se queda con nombres viejos.
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import generate_software_doc as gsd  # noqa: E402

OUT_TEX = os.path.join(HERE, "Guia_Calibracion.tex")

# (qué, cómo, parámetros a cambiar). Los nombres entre {llaves} se revisan contra el programa.
CALIBRAR = [
    ("Signos", "tools/robot/api_check.py, páginas Motores, IMU y Línea: avanzar debe dar "
     "encoders +, girar a la derecha rumbo +, nariz arriba pitch +",
     "{LEFT_SIGN} {RIGHT_SIGN} {HEADING_SIGN} {PITCH_SIGN} {LINE_SIGN} (config.py)"),
    ("Distancia", "Menú > Probar recta 50. Medir con cinta lo que avanzó",
     "{slip} = real / 50"),
    ("Orugas", "Prueba de 1 m de mBlock: anotar lo que avanzó cada oruga",
     "{lr_ratio} = izquierda / derecha"),
    ("Giroscopio", "calibrations.py: B (rumbo 0), girar el robot A MANO una vuelta contra una "
     "escuadra y leer la pantalla", "{gyro_scale} = 360 / lectura"),
    ("Giros", "Menú > Probar giro 90. Medir con transportador",
     "se pasa: bajar {turn_kp}; no termina: subir {v_turn_min}"),
    ("Derrape", "calibrations.py, joystick al centro: círculo de 40 cm. Medir el diámetro real",
     "{skid} (más chico o más grande que 40)"),
    ("Línea", "Robot centrado sobre la línea: Menú > Calibrar linea. Revisar con Ver sensores "
     "(blanco ~0, negro ~1)", "se guarda solo; umbral {dark_th}"),
    ("Garra", "Menú > Garra: con < > cambiar y A para probar",
     "{claw_open} abierta, {claw_cube} cerrada en el cubo"),
    ("Brazo", "Con regla: altura y adelanto del centro de la garra con el servo a 0, 45 y 90",
     "{arm_h0} {arm_x0} {arm_h45} {arm_x45} {arm_h90} {arm_x90}"),
    ("Colores", "api_check.py página Colores sobre cada color de la pista",
     "{color_mode} = 1 y {COLOR_REF} (config.py)"),
]

# (síntoma, qué cambiar)
SINTOMAS = [
    ("Todas las distancias salen cortas o largas", "{slip} (Probar recta 50)"),
    ("La pantalla dice 90° pero en el piso giró otra cosa", "{gyro_scale}"),
    ("Gira de más u oscila al terminar un giro", "bajar {turn_kp}; subir {turn_tol}"),
    ("Se tuerce al avanzar recto", "subir {head_kp}"),
    ("Zigzaguea siguiendo la línea", "bajar {kp}, subir {kd}"),
    ("Se sale de la línea en las curvas", "bajar {v_line}; subir {kp}"),
    ("Pierde la línea en zonas de color o con otra luz", "Calibrar linea otra vez; {dark_th}"),
    ("Cuenta cruces de más (marcas verdes)", "subir {cross_th}"),
    ("No ve un cruce", "bajar {cross_th}; subir {cross_win}"),
    ("La garra queda antes o después del cubo (nivel 1)", "{nivel_1_cruce_a_pedestal_cm}"),
    ("El cubo no queda sobre el pedestal (nivel 2)", "{nivel_2_t_a_pedestal_cm}"),
    ("La garra choca con el pedestal al bajar", "{h_pedestal}"),
    ("Suelta el cubo o no lo aprieta", "{claw_cube}"),
    ("Choca con la pared al girar (nivel 1)", "{nivel_1_x_de_bajada}, {nivel_1_linea_inferior_y}"),
    ("Roza los muros del pasillo (nivel 2)", "{nivel_2_diagonal_punto_x}, "
     "{nivel_2_diagonal_punto_y}; si oscila, bajar {nivel_2_diagonal_kp}"),
    ("Una oruga se sale de la rampa blanca", "{nivel_2_rampa_centro_x}"),
    ("Se queda sin fuerza o resbala en rampa/escalera", "subir {v_ramp} / {v_stairs}"),
    ("Se levanta de la nariz o se voltea al subir", "bajar {tip_slew}; {h_climb}"),
    ("Lee mal los colores", "{color_mode} = 1 y {COLOR_REF}"),
    ("Sigue con un valor viejo aunque cambié el archivo", "lo guardado en el menú manda: "
     "Menú > Restaurar valores y B"),
]


def names_in(text):
    return re.findall(r"\{(\w+)\}", text)


def check_names(cfg, pista):
    bad = []
    for row in CALIBRAR + SINTOMAS:
        for n in names_in(" ".join(row)):
            if n not in cfg.P and not hasattr(cfg, n) and not hasattr(pista, n):
                bad.append(n)
    if bad:
        sys.exit("La guía menciona parámetros que ya no existen: %s" % sorted(set(bad)))


def fmt(text, tune):
    """Texto con {nombres} -> LaTeX; los parámetros del menú llevan una marca."""
    out = []
    pos = 0
    for m in re.finditer(r"\{(\w+)\}", text):
        out.append(gsd.tex(text[pos:m.start()]))
        n = m.group(1)
        out.append(gsd.tt(n) + (r"$^{\scriptscriptstyle M}$" if n in tune else ""))
        pos = m.end()
    out.append(gsd.tex(text[pos:]))
    return "".join(out)


def pista_rows(path):
    """Por nivel: medidas marcadas MEDIR/MEDIDO; las parejas _x/_y van en un renglón."""
    out = []
    for title, _, subs in gsd.pista_sections(path):
        rows = [r for _, rs in subs for r in rs if "MEDI" in r[2]]
        merged = []
        i = 0
        while i < len(rows):
            n, v, s, d = rows[i]
            if n.endswith("_x") and i + 1 < len(rows) and rows[i + 1][0] == n[:-1] + "y":
                merged.append((n[:-1] + "x/y", "%s, %s" % (v, rows[i + 1][1]), s, d))
                i += 2
            else:
                merged.append((n, v, s, d))
                i += 1
        out.append((title, merged))
    return out


def build():
    cfg, _ = gsd.load_program()
    import pista
    check_names(cfg, pista)
    tune = {t[1] for t in cfg.TUNE}
    lines = []
    w = lines.append
    w(r"""% ARCHIVO GENERADO por generate_guia_calibracion.py -- no editar a mano.
\documentclass[9pt]{extarticle}
\usepackage[utf8]{inputenc}
\usepackage[T1]{fontenc}
\usepackage{lmodern}
\usepackage{textcomp}
\usepackage[spanish,es-noshorthands]{babel}
\usepackage[landscape,letterpaper,margin=0.9cm,top=0.8cm,bottom=0.8cm]{geometry}
\usepackage{multicol}
\usepackage{array}
\usepackage{xcolor}
\usepackage{enumitem}
\usepackage{amssymb}
\setlength{\columnsep}{0.6cm}
\setlength{\parindent}{0pt}
\pagestyle{empty}
\renewcommand{\arraystretch}{1.05}
\newcommand{\hd}[1]{\par\smallskip{\color{blue!60!black}\large\bfseries #1}\par\smallskip}
\begin{document}
{\LARGE\bfseries Rover X -- Guía rápida de calibración}\hfill
{\small Orden: \textbf{1} calibrar el robot $\rightarrow$ \textbf{2} medir la pista
$\rightarrow$ \textbf{3} probar y corregir. Cambiar \textbf{una cosa a la vez} y anotarla.}
\par\vspace{2pt}\hrule\vspace{4pt}
{\small Dónde se cambia: los parámetros con $^{\scriptscriptstyle M}$ están en el
\textbf{menú del CyberPi} (programa autónomo, \textbf{centro del joystick}; $\uparrow\downarrow$
elegir, $\leftarrow\rightarrow$ valor, A probar, \textbf{B guardar}). Lo demás, en
\texttt{software/core/config.py} o \texttt{software/core/pista.py}; después correr
\texttt{python3 tools/scripts/bundle\_for\_mblock.py -{}-all} y volver a cargar
\texttt{mblock/rover\_x\_auto.py}.}
\begin{multicols}{3}
\raggedcolumns""")
    # 1. robot
    w(r"\hd{1. Calibrar el robot}")
    w(r"{\footnotesize Una vez, y otra si se cambia algo del robot (orugas, brazo, sensor).}\par")
    w(r"\footnotesize\begin{enumerate}[leftmargin=1.2em,itemsep=2pt,topsep=2pt]")
    for what, how, change in CALIBRAR:
        w(r"\item \textbf{%s.} %s.\\ $\Rightarrow$ %s" % (gsd.tex(what), gsd.tex(how),
                                                       fmt(change, tune)))
    w(r"\end{enumerate}")
    # 2. pista
    w(r"\hd{2. Medir la pista}")
    w(r"\footnotesize Con flexómetro, en cm. Escribir el valor en \texttt{pista.py} y cambiar "
      r"\texttt{MEDIR} por \texttt{MEDIDO}. Mapa: \textbf{x} hacia la derecha desde la orilla "
      r"izquierda, \textbf{y} hacia abajo desde la orilla de arriba (vista superior del "
      r"reglamento). Rumbo 0 = derecha, 90 = abajo. \textbf{Centro de giro} = punto medio de "
      r"las orugas (las sondas van 5.2 cm delante). Cada medida está explicada en "
      r"\texttt{pista.py}.\par\smallskip")
    w(r"\scriptsize")
    for k, (title, rows) in enumerate(pista_rows(os.path.join(gsd.ROOT, "software/core/pista.py"))):
        if k == 0:
            w(r"\columnbreak")            # las tablas de la pista juntas en la columna 2
        w(r"\textbf{%s}\par" % gsd.tex(title))
        w(r"\begin{tabular}{@{}>{\raggedright\arraybackslash}p{0.66\columnwidth}"
          r">{\raggedleft\arraybackslash}p{0.2\columnwidth}l@{}}")
        for n, v, s, _ in rows:
            done = r"\checkmark" if s.startswith("MEDIDO") else r"$\square$"
            w(r"%s%s & %s & %s \\" % (gsd.tt(n.replace("nivel_%s_" % title.split()[1], "")),
                                     r"$^{\scriptscriptstyle M}$" if "menú" in s else "",
                                     gsd.tex(v), done))
        w(r"\end{tabular}\par\smallskip")
    # 3. síntomas
    w(r"\columnbreak")
    w(r"\hd{3. Si pasa esto, cambia esto}")
    w(r"\footnotesize\begin{tabular}{@{}>{\raggedright\arraybackslash}p{0.47\columnwidth}"
      r">{\raggedright\arraybackslash}p{0.5\columnwidth}@{}}")
    w(r"\textbf{Síntoma} & \textbf{Cambiar} \\\hline")
    for sym, change in SINTOMAS:
        w(r"%s & %s \\[1pt]" % (gsd.tex(sym), fmt(change, tune)))
    w(r"\end{tabular}")
    w(r"\hd{4. Antes de competir}")
    w(r"\footnotesize\begin{itemize}[leftmargin=1em,itemsep=1pt,topsep=2pt]")
    w(r"\item Tras cambiar un archivo: \texttt{python3 -m pytest} y probar en el simulador "
      r"(\texttt{python3 simulation/run\_auto.py -{}-level N}).")
    w(r"\item Probar cada tramo (nivel 1 a 4) en la pista: elegir el nivel con el joystick, "
      r"B empieza, A para.")
    w(r"\item Anotar en \texttt{config.py} / \texttt{pista.py} lo que se ajustó en el menú: "
      r"si se borra lo guardado en el robot, vuelven los valores del archivo.")
    w(r"\end{itemize}")
    w(r"\end{multicols}")
    w(r"\end{document}")
    return "\n".join(lines)


def main():
    with open(OUT_TEX, "w", encoding="utf-8") as f:
        f.write(build())
    print("Generado", os.path.relpath(OUT_TEX, gsd.ROOT))
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
    pages = re.search(r"Output written on .*\((\d+) page", log)
    n = int(pages.group(1)) if pages else 0
    print("PDF: %s (%d página%s)" % (os.path.relpath(os.path.join(HERE, "Guia_Calibracion.pdf"),
                                                     gsd.ROOT), n, "" if n == 1 else "s"))
    if n != 1:
        sys.exit("La guía debe caber en UNA hoja: acortar los textos")


if __name__ == "__main__":
    main()
