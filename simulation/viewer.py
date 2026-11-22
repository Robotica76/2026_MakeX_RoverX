"""Reproducción interactiva de una corrida: un solo archivo HTML, sin dependencias."""

import json
from pathlib import Path

TEMPLATE = Path(__file__).with_name("viewer_template.html")


def write_html(res, path):
    data = json.dumps(res, separators=(",", ":"), ensure_ascii=False)
    html = TEMPLATE.read_text(encoding="utf-8").replace("/*__DATA__*/null", data)
    Path(path).write_text(html, encoding="utf-8")
