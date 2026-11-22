# Geometría del robot y ajustes en pista

El software no supone ningún chasis en particular. Todo lo que depende de cómo está armado el
robot vive en `cfg.P` ([config.py](../software/core/config.py)), y las cuentas que lo usan
están en [geometry.py](../software/core/geometry.py). Al cambiar el diseño (orugas, garra,
posición del sensor) **se mide de nuevo; no se cambia el código**.

## Qué medir

La hoja para llenar, con la foto del robot propuesto, está en [medidas_robot.md](medidas_robot.md).

Las distancias hacia adelante (`*_fwd`) se miden desde un **punto de referencia** fácil de
ubicar: el punto naranja sobre la caja del mBot2. El robot gira alrededor del **punto medio de
las orugas a lo largo**, que en este diseño queda adelante de la caja. `ref_rot_fwd` es esa
diferencia, y [geometry.py](../software/core/geometry.py) pasa todas las medidas al centro de
giro (`geometry.fwd`).

```text
 vista superior (adelante = derecha)              vista lateral (brazo horizontal)

  ┌──────── track_len ────────┐                      arm_len     arm_grip_dx
  ████████████████████████████  ▲ belt_w          pivote ●━━━━━━━━━●┄┄┄┄◉ centro de la garra
                                │                         │              ┊ arm_grip_dz
        ●ref──►✛ centro         │ robot_w                 │ arm_pivot_h
        ref_rot_fwd             │                  ───────┴──────────────────── piso
  ████████████████████████████  ▼
        ●ref ─────────► sondas del Quad RGB     sensor_fwd (y card_probe_fwd)
        ●ref ─────────────────► pivote del brazo   arm_pivot_fwd
```

| Clave | Qué es | Para qué se usa |
| --- | --- | --- |
| `ref_rot_fwd` | referencia → punto medio de las orugas | Convertir todas las medidas al centro de giro |
| `sensor_fwd` | referencia → sondas del Quad RGB | Al detectar un cruce, avanzar lo que falta para **girar sobre el cruce** |
| `card_probe_fwd` | referencia → sonda que lee las tarjetas | Regresar exactamente a la tarjeta roja |
| `probe_pitch` | separación entre sondas vecinas | Posición de cada sonda (calibración, pruebas) |
| `track_len`, `belt_w`, `robot_w` | largo de la oruga, ancho de la banda, ancho total | Parte trasera (`track_len/2`), distancia entre orugas, rodear la torre, quedar "completamente dentro" |
| `arm_pivot_fwd`, `arm_pivot_h` | posición del eje del servo del brazo | Alcance y altura de la garra |
| `arm_len` | barras del paralelogramo, eje a eje | Igual |
| `arm_grip_dx`, `arm_grip_dz` | del eje del extremo del brazo al centro de la garra | Igual |
| `claw_tip` | del centro de la garra a su punta | Largo total hacia adelante |
| `sprocket_d`, `slip` | rueda motriz de la oruga y patinaje | Convertir grados del encoder a cm |
| `arm_zero`, `arm_dir` | ángulo del servo con el brazo horizontal; sentido | Convertir **altura en cm → ángulo del servo** |
| `h_floor`, `h_pedestal`, `h_carry`, `h_bar` | alturas de trabajo de la garra | Misiones 2, 4, 8, 12 y el control manual |
| `claw_open`, `claw_cube`, `claw_ball` | ángulos del servo de la garra | Igual |

Derivados (no se miden): alcance de la garra `claw_reach(h) = arm_pivot_fwd − ref_rot_fwd +
arm_len·cos θ + arm_grip_dx`, distancia entre orugas `robot_w − belt_w`, parte trasera
`track_len / 2`, frente = el mayor entre la garra (+ `claw_tip`) y la mitad de la oruga.

## Sin ultrasónico: medidas de la pista

El diseño no lleva sensor de distancia, así que los props se alcanzan **con el encoder, desde
una referencia que las sondas sí detectan**: un cruce o el borde de una zona de color. Para cada
uno se mide una vez en la pista de prueba la distancia de la referencia al prop, y la geometría
del robot hace el resto.

| Clave | Referencia → prop | Misión |
| --- | --- | --- |
| `ped1_from_cross` | cruce frente al cubo → centro del pedestal (nivel 1) | 2 |
| `ped2_from_cross` | cruce antes del pedestal → centro del pedestal (nivel 2) | 4 |
| `ball_ped_dist` | centro de la línea en la marca roja → centro del pedestal de la pelota | 8 |
| `tower_from_edge` | borde de salida de la zona verde → cara cercana de la torre | 10 |

Ejemplo (misión 2): cuando las sondas cierran el cruce, el centro de giro está
`sensor_fwd − ref_rot_fwd` atrás de él. El robot avanza `ped1_from_cross − claw_reach(h_pedestal) + sensor_fwd − (lo ya
avanzado)`, cierra la garra y regresa exactamente lo mismo (`retreat`).

Si después se agrega un ultrasónico, se pone `HAS_US = True` en `config.py`. La condición
`("obstacle", cm)` sigue disponible para las misiones.

### El brazo de paralelogramo

La garra mantiene su orientación y la altura sigue `h = arm_pivot_h + arm_len·sen(θ) + arm_grip_dz`.
`geometry.arm_servo(h)` hace la cuenta inversa, y `geometry.claw_reach(h)` corrige el alcance,
porque al subir o bajar el brazo la garra se acerca al robot. Para calibrar:

1. Mover el brazo hasta que quede **horizontal** y anotar el ángulo del servo en `arm_zero`.
2. Subir el ángulo: si el brazo sube, `arm_dir = 1`; si baja, `arm_dir = -1`.
3. Medir `arm_pivot_h` (eje del servo al piso), `arm_len` (eje a eje de las barras),
   `arm_grip_dx` y `arm_grip_dz` (del eje del extremo del brazo al centro de la garra, con el brazo
   horizontal; `dz` negativo si la garra queda abajo).
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

Acciones: *Calibrar linea*, *Ver sensores* (oscuridad por sonda, offset, pitch, rumbo), *Probar giro 90*, *Probar recta 50* (medir con cinta y ajustar `slip`) y
*Restaurar valores*.

Los parámetros se guardan en el CyberPi (`rover_params.json`) y se cargan al encender; la
pantalla muestra "guardados" o "de fabrica". ⚠️ Falta confirmar que el CyberPi permite escribir
archivos desde MicroPython. Si no, los cambios duran hasta apagarlo, y hay que pasar los valores
buenos a `config.py`.
