# API de MicroPython del mBot2 (modo *Upload* de mBlock)

El mBot2 Rover se programa en el **CyberPi** con MicroPython. **No** es la API del NovaPi
(`novapi`, `from mbuild import gamepad`, `encoder_motor_class`) que usa el proyecto Storm Breaker.

Todo el código del robot llama a estas funciones **solo desde [`software/core/hal.py`](../software/core/hal.py)**.
Si una firma resulta distinta en el robot, se corrige en un solo lugar.

Estado: ✅ visto en ejemplos funcionales de MicroPython · ⚠️ por confirmar en el robot con
[`tools/robot/api_check.py`](../tools/robot/api_check.py).

## Módulos

```python
import cyberpi, mbot2, mbuild, gamepad, time
```

## Chasis (`mbot2`)

| Llamada | Estado | Notas |
| --- | --- | --- |
| `mbot2.drive_power(em1, em2)` | ✅ | -100..100. **EM1 = izquierda, EM2 = derecha. Avanzar = `(100, -100)`**: EM2 va invertido |
| `mbot2.drive_speed(em1, em2)` | ✅ | RPM, misma convención de signos |
| `mbot2.forward(rpm, s)`, `backward`, `turn_left`, `turn_right` | ✅ | Bloqueantes |
| `mbot2.straight(cm)` / `mbot2.turn(grados)` | ✅ | Bloqueantes; `turn(90)` = derecha |
| `mbot2.EM_stop("all")` | ⚠️ | Si falla, se usa `drive_power(0, 0)` |
| `mbot2.EM_get_angle("EM1")`, `mbot2.EM_reset_angle("all")` | ⚠️ | Odometría con encoders |
| `mbot2.servo_set(angulo, "s1")`, `mbot2.servo_get("s1")` | ✅ | Puertos `"s1".."s4"`, `"all"` o número (`3`) |

## Sensor Quad RGB (`mbuild.quad_rgb_sensor`)

| Llamada | Estado | Notas |
| --- | --- | --- |
| `get_offset_track(1)` | ⚠️ | -100..100; -100 = el sensor está a la izquierda de la línea (doc. mBuild) |
| `get_line_sta("all", 1)` | ✅ | 0..15, orden de bits por confirmar |
| `is_line("L1", 1)` | ✅ | Sondas `"L2" "L1" "R1" "R2"` |
| `get_color_sta("L1")` | ✅ | Devuelve el nombre del color |
| `get_red/get_green/get_blue("L1")` | ✅ | |
| `get_light("L1")`, `get_ground_sta("all", 1)` | ✅ | |
| `set_led(color="white")`, `off_led()` | ✅ | |

## Ultrasónico 2

`mbuild.ultrasonic2.get(index=1)` ✅ (cm)

## CyberPi

| Llamada | Estado |
| --- | --- |
| `cyberpi.get_yaw()`, `get_pitch()`, `get_roll()` | ✅ (signo y rango ⚠️) |
| `cyberpi.controller.is_press(n)` con `a b up down left right middle any` | ✅ |
| `cyberpi.timer.get()`, `cyberpi.timer.reset()` | ✅ |
| `cyberpi.display.show_label(txt, tam, x, y, index)` (tam 12/16/24/32, index 0..7) | ✅ |
| `cyberpi.console.println(txt)`, `cyberpi.console.clear()` | ✅ / ⚠️ |
| `cyberpi.led.on(r, g, b, id="all")` (id 1..5) | ✅ |
| `cyberpi.get_rotation("z")`, `cyberpi.reset_rotation(axis="all")` | ✅ (ángulo acumulado) |
| `cyberpi.reset_yaw()` | ⚠️: no se usa; el HAL guarda su propio cero de yaw |
| `cyberpi.console.set_font(16)`, `console.print(txt)` | ✅ |

## Control Bluetooth Makeblock (`gamepad`)

`gamepad.is_key_pressed(k)` ✅ con `Up Down Left Right N1 N2 N3 N4 L1 L2 R1 R2 L_Thumb R_Thumb Start Select`.
`gamepad.get_joystick("Lx"|"Ly"|"Rx"|"Ry")` ✅: **`Ly` positivo = adelante** (ejemplo oficial de manejo:
`drive_speed(Ly + Rx, Rx - Ly)`). Rango probable -100..100 ⚠️.

Fuente: ejemplos de MicroPython de la comunidad (github.com/PerfecXX/mBot2) y la doc. mBuild de
Makeblock. La documentación oficial de Makeblock no se pudo descargar desde aquí; confirmar en el
editor de mBlock.
