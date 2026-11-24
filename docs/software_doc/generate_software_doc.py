#!/usr/bin/env python3
"""Genera el capítulo LaTeX "Software" a partir de los comentarios Doxygen del código.

Uso:
    python3 docs/software_doc/generate_software_doc.py            # -> generated/software.tex
    python3 docs/software_doc/generate_software_doc.py --pdf      # y compila Software.pdf
    python3 docs/software_doc/generate_software_doc.py --sin-codigo --pdf   # sin listados

Lee los bloques Doxygen de cada módulo de software/ (## @file, ## @brief, #  @details,
#  @param, #  @return) y además saca del propio programa:
  - la tabla de misiones (software/autonomous/missions.py),
  - las medidas de la pista por nivel (software/core/pista.py, con sus comentarios),
  - los parámetros ajustables cfg.P (software/core/config.py, con sus comentarios),
  - los botones de los dos programas (constantes de config.py).
Las decisiones de diseño se editan a mano en design_decisions.json.

Hay que volver a correrlo después de cambiar el código: todo se regenera.
Mismo enfoque que el capítulo Software del repositorio 2026_MakeX_Challengue.
"""
import argparse
import ast
import json
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
OUT_DIR = os.path.join(HERE, "generated")
DECISIONS = os.path.join(HERE, "design_decisions.json")

# módulos en el orden del capítulo: (carpeta, [archivos], título de la sección)
MODULES = [
    ("software/robot", ["rover_x_auto.py", "rover_x_manual.py"], "Programas para el robot"),
    ("software/autonomous", ["autonomous.py", "missions.py"], "Autónomo"),
    ("software/manual", ["manual.py"], "Control manual"),
    ("software/controls", ["motion.py", "linefollow.py", "odometry.py", "gripper.py"], "Control"),
    ("software/core", ["hal.py", "config.py", "pista.py", "geometry.py", "colors.py", "settings.py"],
     "Base (core)"),
]
TEST_PROGRAMS = ("software/robot", ["calibrations.py", "funciones_rover.py"])
PY_PATHS = ["software/core", "software/controls", "software/autonomous", "software/manual",
            "software/robot", "tests/fakes"]

# ------------------------------------------------------------------ texto -> LaTeX
LATEX_SPECIAL = {
    "\\": r"\textbackslash{}", "&": r"\&", "%": r"\%", "$": r"\$", "#": r"\#",
    "_": r"\_", "{": r"\{", "}": r"\}", "~": r"\textasciitilde{}", "^": r"\textasciicircum{}",
}
UNICODE = {
    "°": r"\textdegree{}", "→": r"\(\rightarrow\)", "←": r"\(\leftarrow\)", "↑": r"\(\uparrow\)",
    "↓": r"\(\downarrow\)", "↕": r"\(\updownarrow\)", "↔": r"\(\leftrightarrow\)", "≤": r"\(\le\)",
    "≥": r"\(\ge\)", "±": r"\(\pm\)", "×": r"\(\times\)", "·": r"\(\cdot\)", "…": r"\dots{}",
    "—": "---", "–": "--", "²": r"\textsuperscript{2}", "≈": r"\(\approx\)", "≠": r"\(\ne\)",
    "“": "``", "”": "''", "‘": "`", "’": "'", "µ": r"\(\mu\)", "π": r"\(\pi\)", "Δ": r"\(\Delta\)",
    "✓": r"\(\checkmark\)", "⚠": "(!)", "️": "",
}
LATIN = "áéíóúÁÉÍÓÚñÑüÜ¿¡"


def tex(text):
    """Escapa texto normal para LaTeX (incluye símbolos Unicode que pdflatex no conoce)."""
    out = []
    for c in str(text):
        if c in LATEX_SPECIAL:
            out.append(LATEX_SPECIAL[c])
        elif c in UNICODE:
            out.append(UNICODE[c])
        elif ord(c) < 128 or c in LATIN:
            out.append(c)
        else:
            out.append("?")
    return "".join(out)


def tt(text):
    """Código en línea; los nombres largos pueden cortarse en los guiones bajos."""
    return r"\texttt{%s}" % tex(text).replace(r"\_", r"\_\allowbreak{}")


def lit_char(c):
    """Cómo escribe listings un carácter no ASCII del código fuente."""
    acc = {"á": r"\'a", "é": r"\'e", "í": r"\'i", "ó": r"\'o", "ú": r"\'u", "Á": r"\'A",
           "É": r"\'E", "Í": r"\'I", "Ó": r"\'O", "Ú": r"\'U", "ñ": r"\~n", "Ñ": r"\~N",
           "ü": r"\"u", "Ü": r"\"U", "¿": r"?`", "¡": r"!`"}
    if c in acc:
        return acc[c]
    if c in UNICODE:
        return UNICODE[c]
    return "?"


# ------------------------------------------------------------------ bloques Doxygen
def comment_body(line):
    s = line.strip()
    if s.startswith("##"):
        return s[2:].lstrip(" ") if not s[2:3] == " " else s[3:]
    if s.startswith("#  "):
        return s[3:]
    if s.startswith("# "):
        return s[2:]
    return s[1:]


def parse_block(lines):
    """Líneas de comentario (con '#') -> {tag: texto}; @param se acumula en una lista.
    El texto conserva los saltos de línea (para listas y tablas alineadas)."""
    tags = {"param": []}
    cur = None
    for raw in lines:
        body = comment_body(raw)
        m = re.match(r"\s*@(\w+)\s?(.*)", body)
        if m:
            tag, rest = m.group(1), m.group(2)
            if tag == "param":
                tags["param"].append(rest)
                cur = ("param", len(tags["param"]) - 1)
            else:
                tags[tag] = rest
                cur = (tag, None)
            continue
        if cur is None:
            continue
        tag, i = cur
        if tag == "param":
            tags["param"][i] += " " + body.strip()
        else:
            tags[tag] += "\n" + body
    return tags


def block_above(lines, lineno):
    """Bloque Doxygen justo arriba de la línea lineno (1-based): empieza en '##'."""
    i = lineno - 2
    got = []
    while i >= 0 and lines[i].strip().startswith("#"):
        got.append(lines[i])
        if lines[i].strip().startswith("##"):
            return parse_block(list(reversed(got)))
        i -= 1
    return {"param": []}


def file_block(lines):
    got = []
    for l in lines:
        if not l.startswith("#"):
            break
        got.append(l)
    return parse_block(got) if got and got[0].startswith("##") else {"param": []}


def render(text):
    """Texto de @details -> LaTeX: renglones "- x" = lista; renglones con sangría o columnas
    alineadas = bloque de texto fijo (alltt); lo demás, párrafos."""
    if not text:
        return ""
    lines = text.split("\n")
    out, para, items, fixed = [], [], [], []

    def flush_para():
        if para:
            out.append(tex(" ".join(p.strip() for p in para)) + "\n")
            del para[:]

    def flush_items():
        if items:
            out.append(r"\begin{itemize}\setlength\itemsep{0pt}")
            for it in items:
                out.append(r"\item " + tex(it))
            out.append(r"\end{itemize}")
            del items[:]

    def flush_fixed():
        if fixed:
            ind = min(len(l) - len(l.lstrip()) for l in fixed if l.strip())
            out.append(r"{\small\begin{alltt}")
            for l in fixed:
                out.append(tex(l[ind:]))
            out.append(r"\end{alltt}}")
            del fixed[:]

    for l in lines:
        s = l.strip()
        is_item = s.startswith("- ")
        is_fixed = (l.startswith("  ") and not is_item) or re.search(r"\S {3,}\S", l)
        if not s:
            flush_para(); flush_items(); flush_fixed()
        elif is_item:
            flush_para(); flush_fixed()
            items.append(s[2:])
        elif items and l.startswith("    ") and not s.startswith("- "):
            items[-1] += " " + s
        elif is_fixed:
            flush_para(); flush_items()
            fixed.append(l)
        else:
            flush_items(); flush_fixed()
            para.append(l)
    flush_para(); flush_items(); flush_fixed()
    return "\n".join(out)


# ------------------------------------------------------------------ análisis del código
class Module:
    def __init__(self, folder, name):
        self.rel = folder + "/" + name
        self.path = os.path.join(ROOT, self.rel)
        self.name = name[:-3]
        with open(self.path, encoding="utf-8") as f:
            self.src = f.read()
        self.lines = self.src.splitlines()
        self.tree = ast.parse(self.src)
        self.doc = file_block(self.lines)
        self.funcs = [n for n in self.tree.body if isinstance(n, ast.FunctionDef)]
        self.aliases = {}           # alias -> módulo del proyecto
        for n in self.tree.body:
            if isinstance(n, ast.Import):
                for a in n.names:
                    self.aliases[a.asname or a.name] = a.name

    def fdoc(self, fn):
        start = fn.decorator_list[0].lineno if fn.decorator_list else fn.lineno
        return block_above(self.lines, start)


def calls_of(mod, fn, known):
    """Funciones del proyecto que llama fn: [(módulo, función)]."""
    own = {f.name for f in mod.funcs}
    out = []
    for n in ast.walk(fn):
        if not isinstance(n, ast.Call):
            continue
        f = n.func
        key = None
        if isinstance(f, ast.Name) and f.id in own and f.id != fn.name:
            key = (mod.name, f.id)
        elif isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name):
            m = mod.aliases.get(f.value.id)
            if m in known and f.attr in known[m]:
                key = (m, f.attr)
        if key and key not in out:
            out.append(key)
    return out


def label(mod, fn):
    return "fn:%s:%s" % (mod, fn.replace("_", "-"))


# ------------------------------------------------------------------ datos del programa
def trailing_comments(lines, i):
    """Comentario al final de la línea i y los renglones de comentario que la continúan
    (con más sangría que el inicio de la línea)."""
    l = lines[i]
    m = re.search(r"#\s?(.*)$", l)
    text = m.group(1).strip() if m else ""
    col = l.index("#") if m else None
    j = i + 1
    while j < len(lines):
        s = lines[j]
        if s.strip().startswith("#") and col is not None and len(s) - len(s.lstrip()) >= col - 2 \
                and not s.strip().startswith("# ----") and not s.strip().startswith("# ===="):
            text += " " + s.strip()[1:].strip()
            j += 1
        else:
            break
    return text


def pista_sections(path):
    """[(título del nivel, texto de la misión, [(subtítulo, [(nombre, valor, estado, desc)])])]"""
    with open(path, encoding="utf-8") as f:
        lines = f.read().splitlines()
    levels, cur, sub = [], None, None
    for i, l in enumerate(lines):
        m = re.match(r"#\s*-{5,}\s*(NIVEL.*)$", l)
        if m:
            cur = [m.group(1).strip(), "", []]
            levels.append(cur)
            sub = None
            continue
        if cur is not None and not cur[2] and re.match(r"#\s+[^-=]", l) and sub is None:
            cur[1] += (" " if cur[1] else "") + l.lstrip("#").strip()
            continue
        m = re.match(r"#\s*----\s+(.*)$", l)
        if m and cur is not None:
            sub = [m.group(1).strip(), []]
            cur[2].append(sub)
            continue
        m = re.match(r"^(nivel_\w+)\s*=\s*([^#]+?)\s*(#.*)?$", l)
        if m and cur is not None:
            desc = trailing_comments(lines, i)
            state = []
            for k in ("MEDIDO", "MEDIR"):
                if re.match(r"%s\b" % k, desc):
                    state.append(k)
                    desc = desc[len(k):].strip()
                    break
            if desc.startswith("[MENU]"):
                state.append("menú")
                desc = desc[6:].strip()
            if sub is None:
                sub = ["", []]
                cur[2].append(sub)
            sub[1].append((m.group(1), m.group(2).strip(), " ".join(state), desc))
    return levels


def config_params(path):
    """[(grupo, [(clave, valor, comentario)])] del diccionario P de config.py."""
    with open(path, encoding="utf-8") as f:
        lines = f.read().splitlines()
    start = next(i for i, l in enumerate(lines) if l.startswith("P = {"))
    groups, cur = [], None
    i = start + 1
    while i < len(lines) and not lines[i].startswith("}"):
        l = lines[i]
        m = re.match(r"\s*#\s*---\s*(.*)$", l)
        if m:
            title = m.group(1).strip()
            cont = []
            j = i + 1
            while j < len(lines) and re.match(r"\s*#(?!\s*---)", lines[j]):
                cont.append(lines[j].strip()[1:].strip())
                j += 1
            cur = [title, " ".join(cont), []]
            groups.append(cur)
            i = j
            continue
        m = re.match(r'\s*"(\w+)":\s*([^#]+?),?\s*(#.*)?$', l)
        if m and cur is not None:
            cur[2].append((m.group(1), m.group(2).rstrip(","), trailing_comments(lines, i)))
        i += 1
    return groups


def load_program():
    """Importa config y missions del proyecto (con los módulos falsos del mBot2)."""
    for p in reversed(PY_PATHS):
        sys.path.insert(0, os.path.join(ROOT, p))
    import config
    import missions
    return config, missions


# ------------------------------------------------------------------ diagramas TikZ
LAYERS = [
    ("Programas", ["rover_x_auto", "rover_x_manual"]),
    ("Autónomo / manual", ["autonomous", "missions", "manual"]),
    ("Control", ["motion", "linefollow", "odometry", "gripper"]),
    ("Base", ["settings", "colors", "geometry", "pista", "config", "hal"]),
]


def deps_tikz(mods):
    names = {m.name for m in mods}
    pos = {}
    out = [r"\begin{tikzpicture}[>=Stealth,",
           r"  md/.style={draw,rounded corners,fill=blue!8,font=\footnotesize\ttfamily,"
           r"minimum height=0.65cm,minimum width=2.1cm,inner sep=3pt},",
           r"  ly/.style={font=\footnotesize\bfseries,anchor=east}]"]
    for row, (title, ms) in enumerate(LAYERS):
        y = -row * 1.7
        out.append(r"\node[ly] at (-1.6,%.2f) {%s};" % (y, tex(title)))
        ms = [m for m in ms if m in names]
        for col, m in enumerate(ms):
            x = col * 2.5 + (6 - len(ms)) * 1.25
            pos[m] = (x, y)
            out.append(r"\node[md] (%s) at (%.2f,%.2f) {%s};" % (m.replace("_", ""), x, y,
                                                               tex(m)))
    for m in mods:
        for dep in sorted(set(m.aliases.values())):
            if dep in ("config", "hal") and m.name not in ("rover_x_auto", "rover_x_manual"):
                continue                     # casi todos los usan: se dice en el pie de figura
            if dep in pos and m.name in pos and dep != m.name:
                same = pos[dep][1] == pos[m.name][1]
                style = "->,gray!70" + (",bend left=25" if same else "")
                out.append(r"\draw[%s] (%s) to (%s);" % (style, m.name.replace("_", ""),
                                                         dep.replace("_", "")))
    out.append(r"\end{tikzpicture}")
    return "\n".join(out)


# ------------------------------------------------------------------ capítulo
def build(code=True):
    with open(DECISIONS, encoding="utf-8") as f:
        decisions = json.load(f)
    mods = [Module(folder, n) for folder, names, _ in MODULES for n in names]
    tests = [Module(TEST_PROGRAMS[0], n) for n in TEST_PROGRAMS[1]]
    known = {m.name: {f.name for f in m.funcs} for m in mods}
    cfg, ms = load_program()
    out = []
    w = out.append

    chars = sorted({c for m in mods + tests for c in m.src if ord(c) > 127})
    literate = " ".join("{%s}{{%s}}1" % (c, lit_char(c)) for c in chars)
    w("% ARCHIVO GENERADO por generate_software_doc.py -- no editar a mano.")
    w(r"\chapter{Software}\label{chap:software}")
    w(r"\lstset{language=Python,basicstyle=\ttfamily\scriptsize,numbers=left,"
      r"numberstyle=\tiny\color{gray},breaklines=true,frame=single,framerule=0.3pt,"
      r"rulecolor=\color{gray!50},keywordstyle=\color{blue!70!black},"
      r"commentstyle=\color{green!40!black},stringstyle=\color{red!60!black},"
      r"showstringspaces=false,columns=fullflexible,keepspaces=true,inputencoding=utf8,extendedchars=true,literate=%s}" % literate)
    w(tex(decisions.get("intro", "")) + "\n")

    # 1. descripción general
    w(r"\section{Descripci\'on general}")
    w(r"El robot usa \textbf{dos programas}, uno para cada parte de la competencia. Cada uno "
      r"se carga en el CyberPi como \textbf{un solo archivo} (carpeta \texttt{mblock/}), que "
      r"\texttt{tools/scripts/bundle\_for\_mblock.py} arma juntando los m\'odulos de "
      r"\texttt{software/}. El archivo empaquetado solo importa m\'odulos de Makeblock "
      r"(\texttt{cyberpi}, \texttt{mbot2}, \texttt{mbuild}, \texttt{gamepad}) y de MicroPython "
      r"(\texttt{time}, \texttt{math}, \texttt{json}).")
    w("")
    w(r"\begin{longtable}{@{}p{0.27\linewidth}p{0.67\linewidth}@{}}\toprule")
    w(r"\textbf{Archivo} & \textbf{Qu\'e hace} \\\midrule\endhead")
    for folder, names, title in MODULES:
        w(r"\multicolumn{2}{@{}l}{\emph{%s} (%s)} \\" % (tex(title), tt(folder)))
        for n in names:
            m = next(x for x in mods if x.rel == folder + "/" + n)
            w(r"\hyperref[mod:%s]{%s} & %s \\" % (m.name.replace("_", "-"), tt(n),
                                                 tex(m.doc.get("brief", "").replace("\n", " "))))
    w(r"\bottomrule\end{longtable}")
    w(r"\begin{figure}[H]\centering\resizebox{\linewidth}{!}{%")
    w(deps_tikz(mods))
    w(r"}\caption{Dependencias entre m\'odulos, generado del c\'odigo: una flecha va del "
      r"m\'odulo que importa al importado. Casi todos usan \texttt{config} y \texttt{hal}; esas "
      r"flechas solo se dibujan desde los programas.}\end{figure}")

    # 2. programas y botones
    w(r"\section{Programas para el robot y botones}")
    lk = {v: k for k, v in cfg.LEVEL_KEYS.items()}
    names = {"up": "arriba", "down": "abajo", "left": "izquierda", "right": "derecha",
             "middle": "centro", "a": "A", "b": "B"}
    w(r"\subsection{Autónomo: \texttt{mblock/rover\_x\_auto.py}}")
    w(r"\begin{center}\begin{tabular}{@{}ll@{}}\toprule")
    w(r"\textbf{Bot\'on del CyberPi} & \textbf{Acci\'on} \\\midrule")
    for lvl in sorted(lk):
        w(r"Joystick %s & elegir el nivel %d \\" % (names[lk[lvl]], lvl))
    w(r"%s & empezar el tramo del nivel elegido \\" % names[cfg.START_BUTTON])
    w(r"%s & parar el robot (para pedir reinicio) \\" % names[cfg.ABORT_BUTTON])
    w(r"Joystick %s & men\'u de ajustes y calibraci\'on (en la pantalla de inicio) \\"
      % names[cfg.MENU_BUTTON])
    w(r"\bottomrule\end{tabular}\end{center}")
    w(r"\subsection{Manual: \texttt{mblock/rover\_x\_manual.py}}")
    w(r"\begin{center}\begin{tabular}{@{}ll@{}}\toprule")
    w(r"\textbf{Bot\'on del CyberPi} & \textbf{Acci\'on} \\\midrule")
    w(r"%s & empezar el control con el gamepad \\" % names[cfg.START_BUTTON])
    w(r"%s & parar (el robot se detiene y espera otra vez %s) \\"
      % (names[cfg.ABORT_BUTTON], names[cfg.START_BUTTON]))
    w(r"\bottomrule\end{tabular}\end{center}")
    man = next(m for m in mods if m.name == "manual")
    w("Mandos del gamepad (de \\texttt{manual.py}):\n")
    w(render(man.doc.get("details", "")))

    # 3. misiones
    w(r"\section{Misiones del aut\'onomo}")
    w(r"La ruta est\'a en \texttt{missions.py} como una lista de misiones; cada misi\'on es una "
      r"lista de pasos que ejecuta \texttt{autonomous.run\_step()} (secci\'on \ref{%s}). "
      r"\emph{Cr\'itica}: si falla, el tramo se detiene. \emph{Fin de tramo}: el robot se "
      r"detiene ah\'i y el equipo pide reinicio en el siguiente nivel." % label("autonomous", "run_step"))
    w(r"\begin{longtable}{@{}rrrlccr@{}}\toprule")
    w(r"\textbf{Misi\'on} & \textbf{Tramo} & \textbf{Pts} & \textbf{Nombre} & "
      r"\textbf{Cr\'itica} & \textbf{Fin de tramo} & \textbf{Pasos} \\\midrule\endhead")
    for m in ms.MISSIONS:
        w(r"%d & %d & %d & %s & %s & %s & %d \\" % (
            m["id"], m["level"], m["pts"], tex(m["name"]), "s\\'i" if m["critical"] else "",
            "s\\'i" if m.get("end_level") else "", len(m["steps"])))
    w(r"\midrule & & %d & \multicolumn{4}{l}{puntos posibles de estas misiones} \\" % ms.total_points())
    w(r"\bottomrule\end{longtable}")

    # 4. medidas de la pista
    w(r"\section{Medidas de la pista}\label{sec:pista}")
    pmod = next(m for m in mods if m.name == "pista")
    w(render(pmod.doc.get("details", "")))
    for title, text, subs in pista_sections(pmod.path):
        w(r"\subsection{%s}" % tex(title.title().replace("Nivel", "Nivel")))
        if text:
            w(tex(text) + "\n")
        for st, rows in subs:
            if st:
                w(r"\paragraph{%s}" % tex(st))
            w(r"\begin{longtable}{@{}p{0.36\linewidth}rlp{0.42\linewidth}@{}}\toprule")
            w(r"\textbf{Variable} & \textbf{Valor} & \textbf{Estado} & \textbf{Descripci\'on} "
              r"\\\midrule\endhead")
            for n, v, s, d in rows:
                w(r"%s & %s & %s & %s \\" % (tt(n), tex(v), tex(s), tex(d)))
            w(r"\bottomrule\end{longtable}")

    # 5. parámetros ajustables
    w(r"\section{Par\'ametros ajustables (\texttt{cfg.P})}")
    w(r"Valores de \texttt{config.py}. Se pueden cambiar desde el men\'u de ajustes del CyberPi "
      r"(los que est\'an en \texttt{cfg.TUNE}) y se guardan en el robot; lo guardado manda "
      r"sobre el archivo. Marcas: \emph{MEDIR}/\emph{MEDIDO} (del robot real), "
      r"\emph{CALIBRAR}/\emph{CALIBRADO} (con las pruebas), \emph{TODO} (pendiente).")
    tune = {t[1]: t[0] for t in cfg.TUNE}
    for title, text, rows in config_params(os.path.join(ROOT, "software/core/config.py")):
        w(r"\subsection*{%s}" % tex(title[:1].upper() + title[1:]))
        if text:
            w(tex(text) + "\n")
        w(r"\begin{longtable}{@{}p{0.2\linewidth}p{0.17\linewidth}lp{0.47\linewidth}@{}}\toprule")
        w(r"\textbf{Clave} & \textbf{Valor} & \textbf{Men\'u} & \textbf{Descripci\'on} "
          r"\\\midrule\endhead")
        for k, v, d in rows:
            w(r"%s & %s & %s & %s \\" % (tt(k), tt(v), tex(tune.get(k, "")), tex(d)))
        w(r"\bottomrule\end{longtable}")

    # 6. decisiones
    if decisions.get("decisions"):
        w(r"\section{Decisiones de dise\~no}")
        for d in decisions["decisions"]:
            w(r"\medskip\noindent\textbf{%s -- %s}\par" % (tex(d["id"]), tex(d["title"])))
            w(r"\noindent\textbf{Decisi\'on:} %s\par" % tex(d["decision"]))
            w(r"\noindent\textbf{Justificaci\'on:} %s\par" % tex(d["rationale"]))

    # 7. referencia
    def module_ref(m, level="section"):
        sec = r"\section" if level == "section" else r"\subsection"
        sub = r"\subsection" if level == "section" else r"\subsubsection"
        w(r"%s{%s}\label{mod:%s}" % (sec, tt(m.rel), m.name.replace("_", "-")))
        if m.doc.get("brief"):
            w(r"\textbf{%s}" % tex(m.doc["brief"].replace("\n", " ")) + "\n")
        w(render(m.doc.get("details", "")))
        w(r"\noindent Archivo fuente: %s (%d l\'ineas)." % (tt(m.rel), len(m.lines)) + "\n")
        for fn in m.funcs:
            d = m.fdoc(fn)
            args = ", ".join(a.arg for a in fn.args.args)
            w(r"%s{%s}\label{%s}" % (sub, tt("%s(%s)" % (fn.name, args)), label(m.name, fn.name)))
            w(r"\noindent " + tex(d.get("brief", "Sin descripci\u00f3n.").replace("\n", " ")) + "\n")
            if d.get("details"):
                w(render(d["details"]))
            if d["param"]:
                w(r"\par\noindent{\small\begin{tabular}{@{}p{0.2\linewidth}p{0.74\linewidth}@{}}")
                w(r"\textbf{Par\'ametro} & \textbf{Descripci\'on} \\")
                for p in d["param"]:
                    n, _, desc = p.partition(" ")
                    w(r"%s & %s \\" % (tt(n), tex(desc.strip())))
                w(r"\end{tabular}}\par\smallskip")
            if d.get("return"):
                w(r"\noindent\textbf{Devuelve:} %s\par" % tex(d["return"].replace("\n", " ")))
            calls = calls_of(m, fn, known)
            if calls:
                w(r"\noindent\textbf{Llama a:} " + ", ".join(
                    r"\hyperref[%s]{%s}" % (label(a, b), tt(b if a == m.name else a + "." + b))
                    for a, b in calls) + r"\par")
            if code:
                start = fn.decorator_list[0].lineno if fn.decorator_list else fn.lineno
                rel = os.path.relpath(m.path, HERE).replace("\\", "/")
                w(r"\lstinputlisting[firstline=%d,lastline=%d,firstnumber=%d]{%s}" %
                  (start, fn.end_lineno, start, rel))

    w(r"\section{Referencia de los m\'odulos}")
    w(r"Cada funci\'on con su descripci\'on Doxygen, par\'ametros, lo que devuelve, las "
      r"funciones del proyecto que llama%s." % (" y su c\\'odigo" if code else ""))
    for folder, names, title in MODULES:
        w(r"\subsection*{%s}" % tex(title))
        for n in names:
            module_ref(next(x for x in mods if x.rel == folder + "/" + n), "sub")
    w(r"\section{Programas de prueba y calibraci\'on}")
    w(r"Programas sueltos (cada uno ya es un solo archivo; se suben directo desde mBlock) para "
      r"calibrar el movimiento del robot.")
    for m in tests:
        module_ref(m, "sub")
    return "\n".join(out) + "\n"


def find_tex(cmd):
    p = shutil.which(cmd)
    if p:
        return p
    p = os.path.join("/Library/TeX/texbin", cmd)          # MacTeX fuera del PATH
    return p if os.path.exists(p) else None


def main():
    ap = argparse.ArgumentParser(description="Genera el capítulo Software (LaTeX) desde Doxygen.")
    ap.add_argument("--pdf", action="store_true", help="compilar main.tex -> Software.pdf")
    ap.add_argument("--sin-codigo", action="store_true", help="no incluir el código de las funciones")
    a = ap.parse_args()
    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, "software.tex")
    with open(out, "w", encoding="utf-8") as f:
        f.write(build(code=not a.sin_codigo))
    print("Generado", os.path.relpath(out, ROOT))
    if a.pdf:
        pdflatex = find_tex("pdflatex")
        if not pdflatex:
            sys.exit("Falta pdflatex (MacTeX / TeX Live)")
        for _ in range(3):                                 # índice y referencias
            r = subprocess.run([pdflatex, "-interaction=nonstopmode", "-halt-on-error", "main.tex"],
                               cwd=HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            if r.returncode != 0:
                log = r.stdout.decode("utf-8", "replace")
                sys.exit("pdflatex falló:\n" + "\n".join(log.splitlines()[-30:]))
        shutil.copy(os.path.join(HERE, "main.pdf"), os.path.join(HERE, "Software.pdf"))
        print("PDF:", os.path.relpath(os.path.join(HERE, "Software.pdf"), ROOT))


if __name__ == "__main__":
    main()
