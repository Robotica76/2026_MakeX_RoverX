"""La documentación (docs/software_doc) se genera de los comentarios Doxygen: cada módulo de
software/ debe tener su bloque @file y cada función de nivel superior su @brief."""
import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
FILES = sorted(p for p in (ROOT / "software").rglob("*.py") if "__pycache__" not in p.parts)


@pytest.mark.parametrize("path", FILES, ids=lambda p: p.relative_to(ROOT).as_posix())
def test_doxygen_comments(path):
    lines = path.read_text(encoding="utf-8").splitlines()
    assert lines[0].startswith("## @file %s" % path.name), "falta '## @file %s' al inicio" % path.name
    missing = []
    for fn in ast.parse("\n".join(lines)).body:
        if not isinstance(fn, ast.FunctionDef):
            continue
        i = (fn.decorator_list[0].lineno if fn.decorator_list else fn.lineno) - 2
        block = []
        while i >= 0 and lines[i].strip().startswith("#"):
            block.append(lines[i].strip())
            if lines[i].strip().startswith("##"):
                break
            i -= 1
        if not any("@brief" in b for b in block) or not block[-1].startswith("##"):
            missing.append(fn.name)
    assert not missing, "funciones sin bloque '## @brief': %s" % missing
