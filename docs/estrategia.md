# Estrategia del autónomo (borrador)

La ruta está en [`software/autonomous/missions.py`](../software/autonomous/missions.py) como
listas de pasos. Se armó con las vistas de la arena del reglamento y **no se ha probado en
pista**. Los giros, distancias y conteos se ajustan ahí, sin tocar la lógica.

![Arena automática](../resources/reglamento/arena/arena_automatica_iso.png)

| Nivel | Inicio | Recorrido previsto | Primitivas |
| --- | --- | --- | --- |
| 1 | X roja | Red de líneas con marcas verdes y un tope, hasta el pedestal del cubo al pie de la rampa. Tomar el cubo, volver a la línea y subir la rampa | `line` con giros por marcas verdes, `approach` (ultrasónico), garra, `hold` 5 s, `climb` (pitch) |
| 2 | Zona amarilla | Tramo hasta el pedestal: dejar el cubo. Pasillo entre muros sin línea, con recortes sueltos: cruzarlo recto con giroscopio. Rampa blanca | `line … obstacle`, `straight_slow`, `spin` para recuperar la línea, `climb` |
| 3 | Zona azul | Curva en S con el prisma sobre la línea: empujarlo hasta la marca roja y rodearlo. Pedestal de la pelota a un lado. Escalera | `line … color red` a velocidad de empuje, `detour`, `stairs` |
| 4 | Zona verde | Torre fija sobre la línea: rodearla. Tarjetas de color junto a la barra: leerlas y recordar la roja. Regresar a la casilla roja y dejar la pelota. Seguir hasta la X azul y parar | `detour`, `cards`, `drop_red`, `stop_in blue` |

## Decisiones

- **El nivel de inicio se detecta por el color de la zona** (rojo, amarillo, azul o verde).
  Coincide con la regla de reinicio, y también se puede cambiar con el joystick del CyberPi.
- **Misiones críticas y no críticas**: si falla una de navegación (1, 3, 5, 6, 9, 10), el robot
  se detiene; lo conveniente es pedir reinicio en la zona del nivel. Si falla una de
  manipulación, se sigue con la siguiente.
- **Subir rampas y escalera con el pitch del CyberPi**, no por tiempo: se espera a que el robot
  se incline y luego a que quede plano, y se avanzan unos cm más para quedar "completamente
  dentro".
- **Colores por nombre del sensor** (`get_color_sta`, `COLOR_MODE = "builtin"`). Si la luz del
  lugar lo confunde, se cambia a `"nearest"` con la tabla RGB calibrada en `COLOR_REF`.
- **Tarjetas**: se registran el color y la distancia del encoder de cada una. La pelota se deja
  regresando a la distancia de la roja.

## Pendiente de hardware (define constantes)

- Diseño de la garra y del brazo (dos servos, `s1` cierre y `s2` brazo): ángulos `CLAW_*` y `LIFT_*`.
- Altura y posición del Quad RGB y de la sonda que lee las tarjetas (`CARD_PROBE`).
- Ultrasónico al frente, a la altura del pedestal (10 cm) y de la torre.
