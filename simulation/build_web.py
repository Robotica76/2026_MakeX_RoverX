"""Genera el simulador EN VIVO como sitio estático (GitHub Pages): corre en el navegador con
Pyodide, sin servidor de Python. Es el mismo simulador y el mismo programa de competencia que
simulation/live.py (también control manual con teclado o gamepad).

    python3 simulation/build_web.py                  # -> docs/simulador/
    python3 simulation/build_web.py --out build/web  # otra carpeta
    python3 simulation/build_web.py --serve          # y lo sirve en http://127.0.0.1:8780

Hay que volver a generarlo cada vez que cambie el código del robot o del simulador
(rover_py.json lleva una copia de los .py).
"""

import argparse
import json
import shutil
import sys
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / "simulation" / "web"
TEMPLATE = ROOT / "simulation" / "viewer_template.html"

# carpetas con los .py que necesita el simulador en el navegador (rutas como en el repo)
PY_DIRS = ["simulation", "simulation/web", "tests/fakes", "software/core", "software/controls",
           "software/autonomous", "software/manual", "software/robot"]

OVERLAY = """
<style>
  #simload { position: fixed; inset: 0; z-index: 9999; display: flex; align-items: center;
    justify-content: center; background: #1b2028; color: #eceef3; font: 15px sans-serif; }
  #simload div { max-width: 460px; padding: 24px; text-align: center; }
  #simload h1 { font-size: 20px; margin: 0 0 10px; }
  #simload.err p { color: #ff8a80; }
</style>
"""
OVERLAY_DIV = ('<div id="simload"><div><h1>Rover X · simulador</h1>'
               '<p>Preparando el navegador…</p>'
               '<small>La primera vez descarga Python (~10 MB); después queda en caché.</small>'
               '</div></div>')


def build_html():
    html = TEMPLATE.read_text(encoding="utf-8")
    assert "const D = /*__DATA__*/null;" in html
    html = html.replace("const D = /*__DATA__*/null;", "const D = window.__SIM_DATA__;")
    n = html.count("<script>\n")
    assert n == 2, "se esperaban 2 scripts en línea en la plantilla, hay %d" % n
    html = html.replace("<script>\n", '<script type="text/sim-deferred">\n')
    loader = (WEB / "loader.js").read_text(encoding="utf-8")
    html = html.replace("<head>", "<head>\n" + OVERLAY + "<script>\n" + loader + "\n</script>", 1)
    html = html.replace("<body>", "<body>\n" + OVERLAY_DIV, 1)
    return html


def bundle_py():
    files = {}
    for d in PY_DIRS:
        for p in sorted((ROOT / d).glob("*.py")):
            files[p.relative_to(ROOT).as_posix()] = p.read_text(encoding="utf-8")
    return files


class IsolatedHandler(SimpleHTTPRequestHandler):
    """Servidor de prueba: manda COOP/COEP él mismo (en GitHub Pages lo hace coi-sw.js)."""

    def end_headers(self):
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def log_message(self, *a):
        pass


def build(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "index.html").write_text(build_html(), encoding="utf-8")
    for name in ("sim-worker.js", "coi-sw.js"):
        shutil.copy(WEB / name, out / name)
    files = bundle_py()
    (out / "rover_py.json").write_text(json.dumps(files, ensure_ascii=False), encoding="utf-8")
    print("Simulador web en %s (%d archivos .py)" % (out, len(files)))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="Genera el simulador para GitHub Pages")
    ap.add_argument("--out", default=str(ROOT / "docs" / "simulador"))
    ap.add_argument("--serve", action="store_true", help="servirlo localmente para probarlo")
    ap.add_argument("--port", type=int, default=8780)
    a = ap.parse_args(argv)
    out = build(a.out)
    if a.serve:
        srv = None
        for port in range(a.port, a.port + 20):      # si el puerto está ocupado, el siguiente
            try:
                srv = ThreadingHTTPServer(("127.0.0.1", port),
                                          partial(IsolatedHandler, directory=str(out)))
                break
            except OSError:
                print("El puerto %d está ocupado, pruebo el %d" % (port, port + 1))
        if srv is None:
            print("No hay puertos libres entre %d y %d" % (a.port, a.port + 19))
            return 1
        print("Probar en http://127.0.0.1:%d/  (Ctrl+C para salir)" % srv.server_address[1])
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    sys.exit(main())
