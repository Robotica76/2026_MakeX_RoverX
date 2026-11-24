# 2026 MakeX: Rover X

Software del equipo **Robótica 76** para la categoría **Rover X** (CreativaKids / MakeX México):
un **mBot2 con kit Rover** programado en **MicroPython** en el CyberPi.

> **Estado (2026-11-21):** el programa corre en PC con módulos simulados (50 pruebas). **No se
> ha probado en el robot.** La ruta del autónomo es un borrador hecho con las vistas de la arena.

## Qué hace

Son **dos programas**, uno para cada parte de la competencia. Los archivos listos para pegar en
mBlock están en [`mblock/`](mblock):

| Archivo para el robot | Botones del CyberPi |
| --- | --- |
| [`mblock/rover_x_auto.py`](mblock/rover_x_auto.py) (autónomo) | joystick **arriba** = nivel 1, **abajo** = 2, **izquierda** = 3, **derecha** = 4 · **B** = empezar el tramo · **A** = parar · centro = ajustes y calibración |
| [`mblock/rover_x_manual.py`](mblock/rover_x_manual.py) (manual, alianza) | **B** = empezar el control con el gamepad · **A** = parar |

El autónomo corre el tramo del nivel elegido y se detiene al terminarlo; vuelve a la pantalla de
inicio con el siguiente nivel ya elegido.

## Organización

| Carpeta | Contenido |
| --- | --- |
| [software/core/pista.py](software/core/pista.py) | **Medidas de la pista** que usa el autónomo, por nivel (`nivel_1_...` a `nivel_4_...`): lo primero que hay que medir y ajustar en la pista real |
| [software/core](software/core) | `config.py` (constantes y parámetros ajustables `P`), `geometry.py` (cuentas con la geometría del robot), `hal.py` (única capa que llama a la API del mBot2; lectura analógica de la línea), `colors.py`, `settings.py` (menú de ajustes, calibración, guardado) |
| [software/controls](software/controls) | `linefollow.py` (PID, cruces con memoria, giros en cruce, verde opcional), `motion.py` (giros y rectas con giroscopio, rampas, escalera, rodeo), `gripper.py` |
| [software/autonomous](software/autonomous) | `missions.py` (ruta como datos, misiones 1 a 12 y meta), `autonomous.py` (ejecutor y elección de nivel) |
| [software/manual](software/manual) | `manual.py` (Bluetooth Controller) |
| [software/robot](software/robot) | `rover_x_auto.py` (autónomo) y `rover_x_manual.py` (manual): código fuente de los dos programas |
| [mblock](mblock) | Los dos programas en un solo archivo cada uno, listos para mBlock (generados) |
| [tools/robot/api_check.py](tools/robot/api_check.py) | Programa de prueba y calibración para el robot |
| [tools/scripts/bundle_for_mblock.py](tools/scripts/bundle_for_mblock.py) | Junta todos los módulos en un solo archivo para mBlock |
| [simulation](simulation/README.md) | Simulador del área automática: pista del reglamento, física, juez, Monte Carlo y reproducción HTML |
| [Guia_Calibracion.pdf](docs/software_doc/Guia_Calibracion.pdf) | **Guía rápida de calibración** (una hoja para los alumnos): calibrar el robot, medidas de la pista y qué cambiar según el síntoma |
| [docs/software_doc](docs/software_doc/README.md) | Capítulo **Software** en PDF, generado de los comentarios Doxygen del código ([`Software.pdf`](docs/software_doc/Software.pdf)) |
| [tests](tests) | Pruebas en PC con `cyberpi` / `mbot2` / `mbuild` / `gamepad` falsos |
| [docs](docs) | [Resumen del reglamento](docs/reglamento.md), [hoja de medidas del robot](docs/medidas_robot.md), [geometría y ajustes en pista](docs/geometria.md), [estrategia](docs/estrategia.md), [API del mBot2](docs/api_mbot2.md), [control manual](docs/control_manual.md) |
| [resources/reglamento](resources/reglamento) | PDF del reglamento y vistas de la arena |

## Uso

Pruebas en PC:

```bash
python3 -m venv .venv && .venv/bin/pip install pytest
```

```bash
.venv/bin/python -m pytest -q tests
```

Simulador del autónomo (corre el programa real en la pista virtual y abre la reproducción):

```bash
python3 simulation/run_auto.py
```

Cargar en el robot: abrir `mblock/rover_x_auto.py` o `mblock/rover_x_manual.py`, copiar todo y
pegarlo en el editor de Python de mBlock (dispositivo mBot2, modo **Upload**). Esos archivos se
generan; no se editan a mano. Después de cambiar el código fuente hay que regenerarlos
(una prueba avisa si quedaron desactualizados):

```bash
python3 tools/scripts/bundle_for_mblock.py --all
```

## Primeros pasos con el robot

1. Cargar [`tools/robot/api_check.py`](tools/robot/api_check.py) y confirmar las funciones marcadas ⚠️ en
   [docs/api_mbot2.md](docs/api_mbot2.md): encoders, `get_offset_track`, signos del giroscopio y orden de bits de la línea.
2. Corregir los signos de [`config.py`](software/core/config.py) (`HEADING_SIGN`, `PITCH_SIGN`).
3. Medir la geometría del robot y el brazo ([docs/geometria.md](docs/geometria.md)) y ponerla en `P`, o ajustarla desde el menú.
4. En la pista: menú → *Calibrar linea*; ajustar `kp`, `kd` y `v_line`; *Probar recta 50* para `slip`.
5. Medir en la pista las medidas marcadas `MEDIR` en [`pista.py`](software/core/pista.py), nivel por nivel, y regenerar `mblock/`.
6. Contar los cruces del nivel 1 y escribir `ROUTE_L1` en [`missions.py`](software/autonomous/missions.py); ajustar la ruta nivel por nivel.

## Reglas de código

- Solo `hal.py` importa `cyberpi`, `mbot2`, `mbuild` y `gamepad`.
- Lo que se ajusta en pista va en `cfg.P` (y en `TUNE` si debe aparecer en el menú); las misiones lo nombran como texto (`"h_pedestal"`) para leerlo al ejecutar.
- Compatible con MicroPython: sin f-strings ni type hints, usar `"%d" % x`.
- Los módulos del proyecto se importan como `import x` o `import x as y`, nunca con
  `from x import` (lo exige el bundler). Los nombres de módulo no se repiten.
- No asignar a una variable global el mismo nombre de un módulo importado (el bundler la renombra).
