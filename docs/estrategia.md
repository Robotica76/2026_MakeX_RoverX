# Estrategia del autónomo (borrador)

La ruta está en [`software/autonomous/missions.py`](../software/autonomous/missions.py) como
listas de pasos. Se armó con las vistas de la arena del reglamento y **no se ha probado en
pista**. Los giros, distancias y conteos se ajustan ahí, sin tocar la lógica.

![Arena automática](../resources/reglamento/arena/arena_automatica_iso.png)

| Nivel | Inicio | Recorrido previsto | Primitivas |
| --- | --- | --- | --- |
| 1 | X roja | **Por comandos** (experiencia: seguir la línea delgada no funcionaba): "avanza X cm sosteniendo el rumbo Y" con el IMU; cada tramo termina en una línea que cruza el camino (`cross`) para reiniciar el error. Tomar el cubo, ir al pie de la rampa y subirla con control activo | `ref`, `drive`, `cross`, `approach_ped`, garra, `hold` 5 s; en la rampa: seguir la línea (no subir en diagonal), brazo bajo al frente (`h_climb`) y potencia que sube de a poco (`tip_slew`) |
| 2 | Zona amarilla | Tramo hasta el pedestal: dejar el cubo. Pasillo entre muros sin línea, con recortes sueltos: cruzarlo recto con giroscopio. Rampa blanca | `line … junction` + `approach_ped`, `straight` lento, `spin` para recuperar la línea, `climb` |
| 3 (nuevo) | Zona azul | **Por maniobra**, sin seguir la línea: salir de la zona azul, tomar el prisma con la garra abierta y baja, llevarlo a la zona punteada por distancias fijas, retroceder, cruzar la línea de la escalera (alineación lateral) y subir. Pelota omitida | `edge`, `drive`, `head`, `cross`, `crest` |
| 3 (antes) | Zona azul | Curva en S con el prisma sobre la línea: empujarlo hasta la marca roja y rodearlo. Pedestal de la pelota a un lado. Escalera | `line … color red` a velocidad de empuje, `detour`, `approach_obj` (pelota), `stairs` |
| 4 | Zona verde | Torre fija sobre la línea: rodearla. Tarjetas de color junto a la barra: leerlas y recordar la roja. Regresar a la casilla roja y dejar la pelota. Seguir hasta la X azul y parar | `leave green` + `to_tower` (distancia desde la zona), `detour`, `cards`, `drop_red`, `stop_in blue` |

## Programa por tramos y reinicios

Como lo usaba el equipo: en la pantalla de inicio del CyberPi el **joystick elige el nivel**
(↑ 1, ↓ 2, ← 3, → 4; `cfg.LEVEL_KEYS`) y **B (play) arranca** el tramo de ese nivel
(`cfg.START_BUTTON`; A aborta durante el autónomo, A en la pantalla de inicio = control manual,
centro del joystick = ajustes). El robot **se detiene al terminar el tramo** y vuelve a la
pantalla con el siguiente nivel ya elegido: el equipo pide reinicio, lo coloca en la zona de
color y presiona B. Así la condición con la que terminó un nivel no interfiere con el siguiente.

| Tramo | Arranca en | Misiones | Termina |
| --- | --- | --- | --- |
| 1 | X roja | 1, 2, 3, 4 | con el cubo en el pedestal del nivel 2 (reiniciar en amarillo con el cubo en la garra hace que se lo quiten) |
| 2 | Zona amarilla | 5, 6 | completamente dentro del nivel 3 |
| 3 | Zona azul | 7, 9, 10, 11, 13 | en la meta (el robot no cabe entre la escalera y la torre: los niveles 3 y 4 van seguidos) |
| 4 | Zona verde | 10, 11, 13 | en la meta (para reintentar el nivel 4) |

Si un tramo falla, se repite: las misiones de navegación se vuelven a correr y las de
manipulación ya logradas en ese encendido se saltan.

## Zona de obstáculos (camino al nivel 3)

Los recortes de cartón hacen patinar las orugas y no hay línea. Lo que hace el robot:

1. Al terminar la línea fija su posición en el mapa (`pose`).
2. Va a un punto lejos del muro de abajo con **odometría encoders + giroscopio** (`goto`): si el
   giroscopio dice que giró más hacia un lado de lo que dicen los encoders, esa oruga patinó y el
   avance se mide con la otra.
3. Al norte **por tiempo** hasta apoyar las orugas en el muro de arriba (`square`): queda
   escuadrado y con la coordenada y exacta, aunque haya patinado.
4. Retrocede, gira al este y sigue la recta del mapa a 6 cm del muro corrigiendo el desvío lateral
   (`track_line`) hasta ver la diagonal; la toma con el seguidor muy despacio, sin girar en el
   lugar.
5. Rampa al nivel 3: sigue la línea (no sube en diagonal), brazo bajo al frente, potencia que
   sube de a poco y **asistencia de subida** si se frena o resbala; nunca se detiene en la
   pendiente.

Calibrar en pista: `skid` (girar 360° en el lugar: grados de encoder contra giroscopio),
`track_k`, `slope_min_v` y la posición real de los muros y la salida.

## Decisiones

- **Nivel 1 por comandos, no siguiendo la línea** (experiencia previa: con la línea delgada y las
  marcas verdes fallaba). Rectas con rumbo absoluto del IMU que terminan en una línea
  perpendicular; en el simulador, 100 % de 40 corridas con ruido (también con línea de 10 mm).
- **Rampas y escalera con control activo contra volcadura**: brazo bajo y al frente durante la
  pendiente (en el simulador, con el centro de masa a 11 cm y una rampa de 35°, el margen sube de
  1.8 a 3.9 cm), potencia que sube de a poco mientras va inclinado, y seguir la línea o sostener
  el rumbo para no subir en diagonal. El simulador calcula el margen contra volcadura de cada
  corrida (`--ramp-deg` prueba rampas de hasta 40°, `--true com_h=…` un centro de masa distinto).
- **Todo lo que depende del chasis va en `cfg.P`** y se ajusta desde el menú del CyberPi.
- **El nivel de inicio se detecta por el color de la zona** (rojo, amarillo, azul o verde).
  Coincide con la regla de reinicio, y también se puede cambiar con el joystick del CyberPi.
- **Misiones críticas y no críticas**: si falla una de navegación (1, 3, 5, 6, 9, 10), el robot
  se detiene; lo conveniente es pedir reinicio en la zona del nivel. Si falla una de
  manipulación, se sigue con la siguiente.
- **Subir rampas y escalera con el pitch del CyberPi**, no por tiempo: se espera a que el robot
  se incline y luego a que quede plano, y se avanzan unos cm más para quedar "completamente
  dentro".
- **Colores por nombre del sensor** (`get_color_sta`, `color_mode = 0`). Si la luz del
  lugar lo confunde, se cambia a `color_mode = 1` (RGB más cercano) con la tabla RGB calibrada en `COLOR_REF`.
- **Tarjetas**: se registran el color y la distancia del encoder de cada una. La pelota se deja
  regresando a la distancia de la roja.

## Pendiente de hardware (define constantes)

- Medidas del diseño propuesto (orugas largas + brazo de paralelogramo con garra): ver
  [geometria.md](geometria.md). Servos: `s1` garra, `s2` brazo.
- Altura y posición del Quad RGB y de la sonda que lee las tarjetas (`CARD_PROBE`, `card_probe_fwd`).
- El robot **no lleva ultrasónico**: medir en la pista las distancias de referencia de
  [geometria.md](geometria.md#sin-ultrasónico-medidas-de-la-pista).

## Revisión del autónomo (23 nov, con el robot medido)

Corregido (simulador, ruido 1, 20 corridas desde el nivel 1: mediana 1000):
- **Tarjetas**: el rodeo de la torre volvía a la línea con la sonda pasada de las dos primeras
  tarjetas. Ahora vuelve lo más cerca posible y retrocede leyendo (`CARDS_BACK_CM`): 5 de 6.
  La sonda de tarjetas es una central (R1): la de afuera (R2, a 3.0 cm medidos) leía fuera de la
  tarjeta (7 cm de ancho). Una tarjeta repetida contaba 6 y paraba sobre la azul, que se tomaba
  por la meta.
- **Reinicio en el nivel 2**: el giro en la T chocaba con el muro del pasillo (11 de 12 con
  ruido): retrocede `L2_T_BACK_CM` antes de girar.
- **Pasillo**: punto de escuadra a 17 cm del muro de abajo (la nariz barre 16 cm al girar);
  salida por el centro de la abertura con corrección máxima de 15°. 60 % → 70 %.
- **Reinicio en el nivel 4**: la zona verde (12.7 cm) es más chica que el robot (20.3). Colocar
  mirando al oeste con la cola sobre el último escalón (nariz a ~4 cm de la torre).
- **Posición de viaje** del brazo (encender y colocar): servo a 90° (garra sobre la nariz).

Pendiente (medir en pista o decidir):
1. **Pasillo (70 %)**: los dos recortes junto al muro de arriba ocupan 15.3 cm, casi el hueco
   entre orugas (15.35); con su posición real se puede elegir otro punto de escuadra.
2. **Tarjeta cian**: en el plano está a 6 cm de la torre; el robot no la puede leer sin tocarla.
   Depende de dónde estén de verdad las tarjetas (pregunta abierta 2 del reglamento).
3. **Cubo con la garra inclinada**: a altura de carga (17.5 cm, servo ~61°) la garra va ~40°
   hacia arriba; verificar que el cubo no se resbala. La misión 2 pide "≥ 5 cm de la base".
4. **Volcadura en la escalera**: margen mínimo 1.7 cm (brazo bajo al frente al subir).
5. **Giroscopio** (`gyro_scale`) y tabla del brazo: calibrar (`calibrations`).
6. Pelota (misiones 8 y 12, 250 pts) omitida por decisión del equipo.

## Choques en los niveles 1 y 2 (23 nov): holguras con el contorno real

La pista es muy justa para el robot medido (21.35 × 20.3 cm). Girando en el lugar, la nariz barre
16 cm de radio y la cola 13.6. El simulador ahora revisa el **contorno** del robot, no solo el centro,
contra las caras de los niveles más altos y la orilla de la arena. Así salieron choques que antes no contaba.

| Dónde | Qué pasaba | Cambio |
| --- | --- | --- |
| Retrocesos (todos) | `motion._hold` invertía la corrección del giroscopio al ir hacia atrás: el rumbo se iba en vez de corregirse | corrección igual hacia adelante y atrás |
| Nivel 1, cruce inferior | al girar sobre el cruce la nariz entraba 3 cm en la pared del nivel 4 | baja 2.8 cm a la derecha de la línea, gira al ver el cruce (retrocede 1.5) y vuelve a la altura del cubo yendo al este |
| Nivel 1, junto al cubo | ningún giro de más de 10° cabe entre el pedestal y la pared del nivel 4 | retrocede 11 cm y **pivotea** al norte sobre la oruga izquierda |
| Nivel 2, pedestal | la nariz quedaba a 0.3 cm del pedestal | `L2_TO_PED_CM` 9.6 → 8.8 |
| Nivel 2, curva amarilla | la diagonal de la línea baja hasta 12.7 cm de la pared del nivel 3 y la esquina la tocaba | desde la T, por odometría: baja, gira al este en y 43.5 y entra por el centro de la abertura |
| Salida de la zona de obstáculos | un giro en el lugar con una oruga sobre un recorte corría el robot hasta 6 cm (la abertura deja 4 por lado) y chocaba con la punta del muro | después de escuadrar: pivote hacia atrás sobre la oruga izquierda, "S" de dos pivotes de 53° hasta el centro de la abertura, x fija al ver la diagonal y gira ya afuera |
| Dentro de la zona de obstáculos | con las dos orugas sobre recortes la odometría se quedaba hasta 15 cm corta en x y el pivote barría el muro izquierdo | escuadra primero contra el muro derecho (x exacta), retrocede a x 188 y luego escuadra contra el de arriba |
| Rampa blanca | la rampa deja 0.78 cm por lado; el seguidor oscilaba ±4.5° y una oruga salía 1.3 cm por la orilla | se alinea al pie, mide la x con la línea (`fix_line`), se centra en plano y sube por el centro con giroscopio + odometría (sin seguir la línea) |

**Por qué pivotes:** en un pivote la oruga quieta no puede patinar. El centro gira alrededor de ella y
la posición final depende solo del ángulo del giroscopio. En un giro en el lugar, en cambio, una
oruga que patina desplaza el robot sin que los encoders lo vean.

**TODO(pista):** medir y ajustar estas constantes de `missions.py`:
- `L1_START`, `L1_CROSS_CY`
- `T2_PLACED`, `T2_FROM_M4`
- `CORRIDOR_DIAG_X`, `RAMP2_X`, `RAMP2_LINE_X`
- `skid` en `config.py`: define el radio del pivote, 11.9 cm

**Pendiente, nivel 4:** el rodeo de la torre todavía roza la orilla de la arena. Entre la torre y la
orilla hay 25.5 cm, y al girar al oeste la nariz sale unos 5 cm. Hay que rediseñarlo.
El simulador lo marca como choque ("orilla de la arena") pero sin frenar al robot.
