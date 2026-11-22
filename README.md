# 2026 MakeX: Rover X

Software del equipo **Robótica 76** para la categoría **Rover X** (CreativaKids / MakeX México):
un **mBot2 con kit Rover** programado en **MicroPython** en el CyberPi.

> **Estado (2026-11-21):** el programa corre en PC con módulos simulados (25 pruebas). **No se
> ha probado en el robot.** La ruta del autónomo es un borrador hecho con las vistas de la arena.

## Qué hace

```text
encender -> menú: nivel sugerido por el color de la zona (Arr/Abj cambia)
A (señal del juez) -> autónomo desde ese nivel        B = abortar (para pedir reinicio)
fin -> A = control manual (alianza)  ·  B = volver al menú
```

## Organización

| Carpeta | Contenido |
| --- | --- |
| [software/core](software/core) | `config.py` (todas las constantes), `hal.py` (única capa que llama a la API del mBot2), `colors.py` |
| [software/controls](software/controls) | `linefollow.py` (PID, cruces, marcas verdes), `motion.py` (giros y rectas con giroscopio, rampas, escalera, rodeo), `gripper.py` |
| [software/autonomous](software/autonomous) | `missions.py` (ruta como datos, misiones 1 a 12 y meta), `autonomous.py` (ejecutor y elección de nivel) |
| [software/manual](software/manual) | `manual.py` (Bluetooth Controller) |
| [software/robot](software/robot) | `rover_x_2026.py`, programa de competencia |
| [tools/robot/api_check.py](tools/robot/api_check.py) | Programa de prueba y calibración para el robot |
| [tools/scripts/bundle_for_mblock.py](tools/scripts/bundle_for_mblock.py) | Junta todos los módulos en un solo archivo para mBlock |
| [tests](tests) | Pruebas en PC con `cyberpi` / `mbot2` / `mbuild` / `gamepad` falsos |
| [docs](docs) | [Resumen del reglamento](docs/reglamento.md), [estrategia](docs/estrategia.md), [API del mBot2](docs/api_mbot2.md), [control manual](docs/control_manual.md) |
| [resources/reglamento](resources/reglamento) | PDF del reglamento y vistas de la arena |

## Uso

Pruebas en PC:

```bash
python3 -m venv .venv && .venv/bin/pip install pytest
```

```bash
.venv/bin/python -m pytest -q tests
```

Cargar en el robot: generar un solo archivo y pegarlo en el editor de Python de mBlock
(dispositivo mBot2, modo **Upload**):

```bash
python3 tools/scripts/bundle_for_mblock.py software/robot/rover_x_2026.py
```

Se genera `build/rover_x_2026_mblock.py` (no se versiona: siempre se edita el código fuente
y se vuelve a empaquetar).

## Primeros pasos con el robot

1. Cargar [`tools/robot/api_check.py`](tools/robot/api_check.py) y confirmar las funciones marcadas ⚠️ en
   [docs/api_mbot2.md](docs/api_mbot2.md): encoders, `get_offset_track`, signos del giroscopio y orden de bits de la línea.
2. Llenar los valores `CALIBRAR` de [`config.py`](software/core/config.py): signos, `CM_PER_DEG`, `LINE_BIT`, ángulos de servos y colores.
3. Ajustar la ruta de [`missions.py`](software/autonomous/missions.py) nivel por nivel en la pista de prueba.
4. Ajustar el PID de línea (`LINE_KP`, `LINE_KD`) a la velocidad de competencia.

## Reglas de código

- Solo `hal.py` importa `cyberpi`, `mbot2`, `mbuild` y `gamepad`.
- Compatible con MicroPython: sin f-strings ni type hints, usar `"%d" % x`.
- Los módulos del proyecto se importan como `import x` o `import x as y`, nunca con
  `from x import` (lo exige el bundler). Los nombres de módulo no se repiten.
- No asignar a una variable global el mismo nombre de un módulo importado (el bundler la renombra).
