# Hoja de medidas: robot propuesto (orugas largas + brazo de paralelogramo)

![Medidas propuestas](img/medidas_propuestas.webp)

Sensores: **solo el Quad RGB** (línea y color), más el giroscopio del CyberPi y los encoders de
los motores del mBot2. Sin ultrasónico. Servos: `s1` garra, `s2` brazo.

Todo en **cm**. Las medidas hacia adelante salen del **punto naranja** (referencia sobre la caja
del mBot2). Los valores se escriben en `P` de [config.py](../software/core/config.py) o desde el
menú del CyberPi.

## Flechas verdes (chasis)

| Flecha en la foto | Clave | Valor |
| --- | --- | --- |
| Vista superior A: hueco entre las orugas (borde interior a borde interior) | (`robot_w` = A + 2B) | **15.35** |
| Vista superior B: ancho de la banda | `belt_w` | **3.0** |
| Ancho total (calculado) | `robot_w` | **21.35** |
| Vista lateral C: eje rueda blanca (atrás) → eje 1a rueda negra (tramo apoyado) | `track_c` | **12.1** |
| Vista lateral D: eje 1a negra → eje 2a negra (nariz levantada) | `track_d` | **4.85** |
| Inclinación de D sobre el piso (estimada de la foto, falta medir) | `nose_deg` | ~45° |
| Diámetro de las ruedas (blancas y negras iguales) | `wheel_d`, `sprocket_d` | **3.95** |
| Grosor de la banda (estimado de la foto, falta medir) | `belt_t` | ~0.4 |

| Vista superior E: borde interior de la oruga izquierda → brazo | (`arm_lat`) | **5.97** |
| Vista superior F: ancho del brazo | `arm_w` | **4.9** |
| Brazo corrido a la izquierda (calculado: E + F/2 − A/2) | `arm_lat` | **0.745** |

Con esto: largo de la oruga visto desde arriba ≈ 20.3 cm (8.4 detrás del centro de giro y
11.85 delante, hasta la nariz); nariz 3.4 cm más alta que el tramo apoyado. El centro de giro
es el punto medio del tramo apoyado (entre los ejes de C).

## Flechas naranjas (desde la referencia)

| Flecha en la foto | Clave | Valor |
| --- | --- | --- |
| Corta, hacia atrás desde el punto: ⚠️ ¿hasta dónde? (¿parte trasera de la caja?) | (solo informativo) | |
| Larga, de abajo: punto → sondas del Quad RGB (barra negra bajo el chasis) | `sensor_fwd`, `card_probe_fwd` | |
| Larga, de arriba: punto → eje del servo del brazo | `arm_pivot_fwd` | |
| Vertical en el pivote: ⚠️ ¿altura del eje del servo sobre el piso? | `arm_pivot_h` | |
| Vertical en el extremo del brazo: ⚠️ ¿altura del eje del extremo? ¿o de la garra? | (ver abajo) | |

## Faltan en la foto

| Medida | Clave | Valor |
| --- | --- | --- |
| **Punto naranja → punto medio de las orugas** (a lo largo). Es la más importante: el robot gira alrededor de ese punto, no de la caja | `ref_rot_fwd` | |
| Largo de las barras del brazo, **eje a eje** | `arm_len` | |
| Del eje del extremo del brazo al centro de la garra: horizontal / vertical (con el brazo horizontal) | `arm_grip_dx` / `arm_grip_dz` | |
| Del centro de la garra a su punta | `claw_tip` | |
| Separación entre dos sondas vecinas del Quad RGB | `probe_pitch` | |
| Ángulo del servo con el brazo **horizontal**, y si al subir el ángulo el brazo sube (+1) o baja (−1) | `arm_zero`, `arm_dir` | |
| Ángulos de la garra: abierta / con cubo / con pelota | `claw_open`, `claw_cube`, `claw_ball` | |

## Cómo comprobar `ref_rot_fwd` en la pista

Poner una cinta en el piso bajo el punto naranja y, desde el menú, usar *Probar giro 90* varias
veces. El punto que **no se mueve** al girar es el centro de giro; su distancia al punto naranja
es `ref_rot_fwd` (positiva si está adelante). Con orugas, el centro real puede correrse hacia
donde está el peso: el valor medido así es el bueno.

## Más medidas (22 nov)

| Medida | Clave | Valor |
| --- | --- | --- |
| G: piso → eje fijo de arriba del brazo | `arm_pivot_h` | **12.2** |
| Sondas del Quad RGB delante del eje de los motores (rueda blanca) | `sensor_fwd` | **11.25** (= 5.2 delante del centro de giro) |
| Entre las dos sondas centrales | `probe_pitch` | **1.25** |
| De sonda central a sonda de afuera | `probe_outer` | **2.4** |
| Prueba "avanza 100 cm" de mBlock: izquierda / derecha | `sprocket_d`, `lr_ratio` | 63.5 / 65 → 4.18 cm, 0.977 |

**Brazo**: fotos con el servo a 0°, 45° y 90°. No es paralelogramo (la garra gira; a 90° apunta
hacia arriba sobre la nariz). Se describe con la tabla `arm_h0/arm_x0`, `arm_h45/arm_x45`,
`arm_h90/arm_x90`: altura del centro de la garra (entre los dedos) sobre el piso y cuánto va
delante de la nariz. **Falta medir con regla** esas seis distancias (hoy son estimadas de las fotos).

**Giroscopio** (`gyro_scale`, falta): girar el robot A MANO 360° contra una escuadra y leer
`cyberpi.get_rotation("z")`. gyro_scale = 360 / lectura. El programa de prueba del equipo usa
1.3; si la escala real es 1.3 y el programa usa 1.0, en el simulador no saca puntos.
