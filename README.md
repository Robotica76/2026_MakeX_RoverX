# 2026 MakeX: Rover X

Software del equipo **Robótica 76** para la categoría **Rover X** (CreativaKids / MakeX México):
un **mBot2 con kit Rover** programado en **MicroPython** en el CyberPi.

> **Estado (2026-11-21):** el programa corre en PC con módulos simulados (50 pruebas). **No se
> ha probado en el robot.** La ruta del autónomo es un borrador hecho con las vistas de la arena.

## Qué hace

```text
encender -> carga parámetros guardados -> menú: nivel sugerido por el color de la zona
            (Arr/Abj cambia · Derecha = ajustes y calibración de línea)
A (señal del juez) -> autónomo desde ese nivel        B = abortar (para pedir reinicio)
fin -> A = control manual (alianza)  ·  B = volver al menú
```

## Organización

| Carpeta | Contenido |
| --- | --- |
| [software/core](software/core) | `config.py` (constantes y parámetros ajustables `P`), `geometry.py` (cuentas con la geometría del robot), `hal.py` (única capa que llama a la API del mBot2; lectura analógica de la línea), `colors.py`, `settings.py` (menú de ajustes, calibración, guardado) |
| [software/controls](software/controls) | `linefollow.py` (PID, cruces con memoria, giros en cruce, verde opcional), `motion.py` (giros y rectas con giroscopio, rampas, escalera, rodeo), `gripper.py` |
| [software/autonomous](software/autonomous) | `missions.py` (ruta como datos, misiones 1 a 12 y meta), `autonomous.py` (ejecutor y elección de nivel) |
| [software/manual](software/manual) | `manual.py` (Bluetooth Controller) |
| [software/robot](software/robot) | `rover_x_2026.py`, programa de competencia |
| [tools/robot/api_check.py](tools/robot/api_check.py) | Programa de prueba y calibración para el robot |
| [tools/scripts/bundle_for_mblock.py](tools/scripts/bundle_for_mblock.py) | Junta todos los módulos en un solo archivo para mBlock |
| [simulation](simulation/README.md) | Simulador del área automática: pista del reglamento, física, juez, Monte Carlo y reproducción HTML |
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
2. Corregir los signos de [`config.py`](software/core/config.py) (`HEADING_SIGN`, `PITCH_SIGN`).
3. Medir la geometría del robot y el brazo ([docs/geometria.md](docs/geometria.md)) y ponerla en `P`, o ajustarla desde el menú.
4. En la pista: menú → *Calibrar linea*; ajustar `kp`, `kd` y `v_line`; *Probar recta 50* para `slip`.
5. Medir en la pista las distancias de referencia a los pedestales y a la torre (menú → *Pista*).
6. Contar los cruces del nivel 1 y escribir `ROUTE_L1` en [`missions.py`](software/autonomous/missions.py); ajustar la ruta nivel por nivel.

## Reglas de código

- Solo `hal.py` importa `cyberpi`, `mbot2`, `mbuild` y `gamepad`.
- Lo que se ajusta en pista va en `cfg.P` (y en `TUNE` si debe aparecer en el menú); las misiones lo nombran como texto (`"h_pedestal"`) para leerlo al ejecutar.
- Compatible con MicroPython: sin f-strings ni type hints, usar `"%d" % x`.
- Los módulos del proyecto se importan como `import x` o `import x as y`, nunca con
  `from x import` (lo exige el bundler). Los nombres de módulo no se repiten.
- No asignar a una variable global el mismo nombre de un módulo importado (el bundler la renombra).
