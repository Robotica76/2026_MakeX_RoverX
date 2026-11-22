# Geometría del robot y ajustes en pista

El software no supone ningún chasis en particular. Todo lo que depende de cómo está armado el
robot vive en `cfg.P` ([config.py](../software/core/config.py)), y las cuentas que lo usan
están en [geometry.py](../software/core/geometry.py). Al cambiar el diseño (orugas, garra,
posición del sensor) **se mide de nuevo; no se cambia el código**.

## Qué medir (vista lateral y superior)

Todas las distancias van en **cm** y se miden desde el **centro de giro**: el punto medio entre
las dos orugas, a la mitad de su largo de contacto con el piso. Ese es el punto alrededor del
cual gira el robot.

```text
 vista superior                                  vista lateral (brazo horizontal)

        |<---- robot_w ---->|                            arm_len
   ┌────┬───────────────┬────┐                     pivote ●━━━━━━━━━● garra
   │ o  │               │ o  │ ▲                          │           ┊ arm_grip_dz
   │ r  │       ✛ centro│ r  │ │ rear_len                  │ arm_pivot_h
   │ u  │       de giro │ u  │ ▼                           │
   │ g  │<─ track_w ─>  │ g  │                      ───────┴──────── piso
   │ a  │               │ a  │
   └────┴───────────────┴────┘
          ▲ sensor_fwd: hasta las sondas del Quad RGB (L2 L1 R1 R2, separadas probe_pitch)
          ▲ us_fwd: hasta la cara del ultrasónico
          ▲ claw_fwd: hasta el punto de agarre, con el brazo horizontal
          ▲ card_probe_fwd: hasta la sonda que lee las tarjetas (CARD_PROBE)
          robot_len: largo total
```

| Clave | Qué es | Para qué se usa |
| --- | --- | --- |
| `sensor_fwd` | centro de giro → sondas del Quad RGB | Al detectar un cruce, avanzar esto para **girar sobre el cruce** |
| `probe_pitch` | separación entre sondas vecinas | Posición de cada sonda (calibración, pruebas) |
| `us_fwd` | centro de giro → ultrasónico | Lectura a la que hay que detenerse frente al pedestal |
| `claw_fwd` | centro de giro → punto de agarre (brazo horizontal) | Alcance de la garra |
| `card_probe_fwd` | centro de giro → sonda de tarjetas | Regresar exactamente a la tarjeta roja |
| `track_w`, `robot_w`, `robot_len`, `rear_len` | dimensiones | Rodear la torre, quedar "completamente dentro" de un nivel o zona |
| `sprocket_d`, `slip` | rueda motriz de la oruga y patinaje | Convertir grados del encoder a cm |
| `arm_pivot_h`, `arm_len`, `arm_grip_dz` | brazo de paralelogramo | Convertir **altura en cm → ángulo del servo** |
| `arm_zero`, `arm_dir` | ángulo del servo con el brazo horizontal; sentido | Igual |
| `h_floor`, `h_pedestal`, `h_carry`, `h_bar` | alturas de trabajo de la garra | Misiones 2, 4, 8, 12 y el control manual |
| `claw_open`, `claw_cube`, `claw_ball` | ángulos del servo de la garra | Igual |

### El brazo de paralelogramo

La garra mantiene su orientación y la altura sigue `h = arm_pivot_h + arm_len·sen(θ) + arm_grip_dz`.
`geometry.arm_servo(h)` hace la cuenta inversa, y `geometry.claw_reach(h)` corrige el alcance,
porque al subir o bajar el brazo la garra se acerca al robot. Para calibrar:

1. Mover el brazo hasta que quede **horizontal** y anotar el ángulo del servo en `arm_zero`.
2. Subir el ángulo: si el brazo sube, `arm_dir = 1`; si baja, `arm_dir = -1`.
3. Medir `arm_pivot_h` (eje del servo al piso), `arm_len` (eje a eje de las barras) y
   `arm_grip_dz` (del extremo del brazo al centro de la garra; negativo si queda abajo).
4. Comprobar con el menú: ir a `h_pedestal` y presionar **A**. La garra debe quedar a esa altura.

## Por qué cambió la lectura de la línea

La línea mide 20 mm ±50%, así que puede tener **apenas 10 mm**. Esto pasaba antes:

- La línea queda **entre** dos sondas, ninguna llega al umbral binario y el robot "pierde" la
  línea o el offset salta, así que zigzaguea.
- Las **marcas verdes** están junto a la línea. Con el robot zigzagueando pasan por fuera de las
  sondas exteriores, y además el color se lee más lento que la línea.

Esto es lo que se hace ahora:

1. **Lectura analógica calibrada** (`line_mode = 1`): se usa la luz de cada sonda normalizada
   entre su blanco y su negro. Una sonda medio cubierta da 0.5, y el **centro ponderado** de las
   cuatro da un offset suave aunque la línea quede entre sondas.
2. **Calibración en la pista** (menú → *Calibrar linea*): con el robot centrado sobre la línea,
   gira ±45° y guarda el blanco y el negro de cada sonda. Conviene hacerlo **al llegar a cada
   mesa y si cambia la luz**.
3. **Cruces con memoria** (`cross_win`): basta con que cada sonda exterior haya visto negro
   dentro de unos cm de avance, no al mismo tiempo.
4. **Giros por ruta**: el nivel 1 se recorre con una tabla de cruces y giros (`ROUTE_L1` en
   [missions.py](../software/autonomous/missions.py)), no con el verde. Al cerrar un cruce, el
   robot avanza lo que falta de `sensor_fwd` y gira sobre él. Las marcas verdes quedan como
   opción (`green = 1`).

Recomendación de hardware: probar alturas del Quad RGB con [api_check](../tools/robot/api_check.py)
(página *Línea*) y elegir la que dé **más contraste** entre blanco y negro con la línea entre
L1 y R1.

## Menú de ajustes en el CyberPi

En el menú de inicio: **joystick a la derecha**.

| Botón | Acción |
| --- | --- |
| arriba / abajo | Elegir parámetro o acción |
| izquierda / derecha | Bajar / subir el valor |
| A | Probar: el brazo va a la altura, la garra a la apertura, o se ejecuta la acción |
| B | Guardar y salir |

Acciones: *Calibrar linea*, *Ver sensores* (oscuridad por sonda, offset, ultrasónico, pitch,
rumbo), *Probar giro 90*, *Probar recta 50* (medir con cinta y ajustar `slip`) y
*Restaurar valores*.

Los parámetros se guardan en el CyberPi (`rover_params.json`) y se cargan al encender; la
pantalla muestra "guardados" o "de fabrica". ⚠️ Falta confirmar que el CyberPi permite escribir
archivos desde MicroPython. Si no, los cambios duran hasta apagarlo, y hay que pasar los valores
buenos a `config.py`.
