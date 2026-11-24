# Documentación del software (capítulo LaTeX)

Genera el capítulo **Software** ([`Software.pdf`](Software.pdf)) a partir de los comentarios Doxygen
del código de `software/` y de los datos del propio programa. Mismo enfoque que el capítulo
Software del repositorio del Challengue.

```bash
python3 docs/software_doc/generate_software_doc.py                 # generated/software.tex
python3 docs/software_doc/generate_software_doc.py --pdf           # y Software.pdf (pdflatex)
python3 docs/software_doc/generate_software_doc.py --sin-codigo --pdf   # sin listados de código
```

Hay que volver a correrlo después de cambiar el código: todo se regenera.
Requiere `pdflatex` (MacTeX o TeX Live); en macOS lo busca también en `/Library/TeX/texbin`.

## Guía rápida de calibración (una hoja)

[`Guia_Calibracion.pdf`](Guia_Calibracion.pdf): hoja para los alumnos con 1) cómo calibrar el
robot, 2) qué medidas de la pista hacen falta (lista sacada de `pista.py`, con casillas) y
3) qué cambiar según lo que haga mal el robot.

```bash
python3 docs/software_doc/generate_guia_calibracion.py
```

Los textos de las partes 1 y 3 están al inicio de ese script (`CALIBRAR`, `SINTOMAS`). El
script falla si mencionan un parámetro que ya no existe o si la hoja ya no cabe en una página.

## Qué contiene el capítulo Software

| Sección | De dónde sale |
| --- | --- |
| Descripción general, figura de dependencias | `@file` / `@brief` de cada módulo e `import` del código |
| Programas y botones | constantes de `config.py` (`LEVEL_KEYS`, `START_BUTTON`, `ABORT_BUTTON`, `MENU_BUTTON`) y el `@details` de `manual.py` |
| Misiones del autónomo | `missions.MISSIONS` |
| Medidas de la pista | `pista.py` (secciones por nivel y el comentario de cada variable) |
| Parámetros ajustables | diccionario `P` de `config.py` (comentarios) y `cfg.TUNE` (menú) |
| Decisiones de diseño | [`design_decisions.json`](design_decisions.json) (se edita a mano) |
| Referencia de los módulos | bloques Doxygen de cada función, llamadas entre funciones y su código |

## Cómo comentar el código

Cada archivo empieza con un bloque `@file` y cada función de nivel superior lleva su bloque
arriba del `def` (una prueba, `tests/test_docs.py`, revisa que no falte ninguno):

```python
## @brief Avanza o retrocede una distancia medida con encoders.
#  @details Sin target se sostiene el rumbo con el que empezó.
#  @param cm Distancia: positiva = adelante, negativa = atrás.
#  @param speed % de potencia (por defecto v_straight).
#  @return True si recorrió la distancia.
def straight(cm, speed=None, timeout=None, target=None):
```

En `@details`, los renglones que empiezan con `- ` salen como lista y los renglones con columnas
alineadas salen tal cual (texto fijo).

Se usan comentarios (`#`) y no docstrings: al empaquetar para mBlock los comentarios se quitan,
así el archivo que se sube al CyberPi no crece con la documentación.

Archivos editables a mano: `design_decisions.json` y `main.tex`. No editar `generated/`.
