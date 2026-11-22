# Simulador del área automática

Corre el **programa real** del robot (`software/`) sobre una pista virtual de Rover X, sin el
robot. Sirve para probar rutas, parámetros y geometría antes de ir a la pista, y para medir con
Monte Carlo qué tan robusta es cada misión.

```bash
python3 simulation/run_auto.py                         # nivel 1, sin ruido; abre el reporte
python3 simulation/run_auto.py --level 3 --noise 1 --seed 7
python3 simulation/run_auto.py --level 2 --runs 40 --noise 1   # % de éxito por misión
python3 simulation/run_auto.py --set v_line=55 --set kp=0.6    # probar parámetros de cfg.P
python3 simulation/run_auto.py --true ref_rot_fwd=6    # el robot real difiere de lo configurado
python3 simulation/run_auto.py --line-w 1.0            # línea de 10 mm (mínimo del reglamento)
python3 simulation/run_auto.py --ramp-deg 40 --true com_h=11   # rampas de 40° y centro de masa alto
```

Solo usa la biblioteca estándar de Python. Cada corrida escribe `build/sim/rover_sim.html`: una
reproducción con la pista, el robot, sus sondas, la garra, el rastro, los eventos y la tabla
**juez vs. programa**, una **vista 3D** (three.js desde un CDN: niveles a su altura, rampas,
escalera de dos tramos, props, robot con pitch, brazo y garra; cámara libre o siguiendo al
robot), una **vista lateral** que sigue al robot a escala real (perfil del
piso con rampas y escalera, robot inclinado con su pitch, brazo y garra a su altura) y la
altura del robot en el tiempo. Clic en un evento, en una misión o en la gráfica de altura
lleva a ese momento; barra espaciadora = reproducir.

## Cómo funciona

| Archivo | Qué hace |
| --- | --- |
| [arena.py](arena.py) | Pista reconstruida de la vista superior del reglamento (1 px = 0.235 cm): líneas, zonas, marcas, alturas de niveles, rampas, escalera de dos tramos, muros del pasillo, props y tarjetas |
| [physics.py](physics.py) | Orugas (zona muerta, retardo, patinaje, derrape al girar, carga en pendiente), encoders, giroscopio, pitch por la altura del piso, **volcadura** por centro de masa + aceleración, Quad RGB (luz, color y RGB por sonda), brazo y garra, objetos, empujar el prisma, choques y caídas |
| [referee.py](referee.py) | Juez independiente: revisa cada misión con las condiciones del reglamento |
| [run_auto.py](run_auto.py) | Corrida, reporte, Monte Carlo y reproducción HTML |

El programa usa los módulos falsos de `tests/fakes` (cyberpi, mbot2, mbuild, gamepad);
`hal.wait()` avanza el reloj virtual, así que 5 minutos de autónomo corren en menos de un
segundo. Con `--noise 1` cada semilla cambia la velocidad de los motores, el patinaje, el
derrape, la deriva del giroscopio, la luz ambiente y el ruido de las sondas. Los recortes del
pasillo y el tope desvían el rumbo al pisarlos.

## Lo que ya encontró (y se corrigió en el programa)

| Hallazgo | Corrección |
| --- | --- |
| Las marcas verdes son oscuras y adelantaban ~2.5 cm cada cruce | `cross_th`: las sondas exteriores piden negro "fuerte" para contar un cruce |
| Con las sondas cerca del centro de giro, tras girar en un cruce ya están sobre la línea nueva y `spin_to_line` la "salía" | `spin_to_line(leave=False)` en los cruces |
| La odometría con `abs()` cambiaba en los giros en el lugar | avance con signo |
| No se puede girar sobre el cruce frente a cada pedestal: el robot barre ~13 cm | retroceder antes de girar (misiones 3 y 4) |
| **Zona 5**: con giros relativos el error de cada giro se arrastraba y el robot chocaba con los muros (2 % de 40 corridas con ruido) | pasos `ref` / `drive`: cada tramo sostiene un **rumbo absoluto** con el IMU, referido a la recta de la rampa → **90 %** |
| `line_end` se disparaba en curvas cerradas | exige 2 cm sin línea |
| El otro brazo de una T contaba como un cruce nuevo después de girar | el seguimiento empieza desarmado tras un `jturn` |
| La curva del nivel 3 empieza dentro de la zona azul (oscura) y se perdía la línea | `offset_from` resta el fondo |
| Relanzar el autónomo sin apagar dejaba referencias viejas | `autonomous.reset_state()` limpia todo |
| `detour` giraba hacia el lado equivocado al volver a la línea | gira hacia el lado del rodeo |
| **Prisma (M7)**: empujado con el frente se sale de la línea en la primera curva, y para empujarlo hacia la zona el robot no cabe detrás | se **lleva con la garra abierta y baja** (`h_prism`), como cuna, siguiendo la línea; en la primera marca roja queda dentro de la zona punteada |
| `turn_to` podía girar por el lado largo (239° en vez de 121°) y la recta sostenía un rumbo a 360° | camino corto (`wrap180`) y `map_target` para `head` / `drive` / `seek` |
| La segunda marca roja va junto a la línea: solo la ve la sonda exterior | aproximación a `v_slow` |
| **Escalera (M9)**: entre la escalera y la torre hay ~20 cm y el robot mide ~24 cm; esperar a quedar plano lo chocaba con la torre (a 100 % la confirmación de 0.4 s son 18 cm) | paso `crest`: sube sosteniendo el rumbo oeste con el IMU y se detiene a una distancia fija desde que ve la zona verde |
| En la escalera cada oruga debe ir sobre su tramo (12 cm de hueco): 4° de error de rumbo bastaban para salirse | `line_ref`: la referencia se toma **promediando el rumbo** en la recta que llega a la escalera |
| La línea pasa por encima de las zonas de color: las sondas centrales leen negro | los colores de zona se buscan con las cuatro sondas (`leave_color`, `crest`) |
| `seek_line` releía la línea para confirmar y con ruido fallaba | usa la lectura que detuvo el avance |
| **Nivel 1 por comandos** (pedido del equipo): con rectas por odometría el cubo solo se tomaba en 62 % (error acumulado en ~1 m) | cada recta termina en una línea perpendicular (`cross`): 100 % de 40 corridas, también con línea de 10 mm |
| **Volcadura**: la subida gradual de potencia casi no cambia el margen; lo que manda es la inclinación con la altura del centro de masa | brazo bajo y al frente en cada pendiente (`h_climb`): margen 1.8 → 3.9 cm (centro de masa a 11 cm, rampa 35°). Recortar la potencia en pendiente fuerte dejaba al robot parado a media rampa de 40°: se quitó. Limitar la potencia de cada oruga frenaba la dirección: solo se limita el avance |
| Rodeo de la torre: el giro en el lugar barría la torre (~13 cm de radio) o volvía sobre una tarjeta | `TOWER_DETOUR` con rumbos absolutos, regreso entre dos tarjetas |

## Estrategia que se simula

- **Reinicio en cada zona de inicio** (estrategia del equipo): al completar un nivel el robot pide
  reinicio en la zona de color del siguiente; el simulador hace de equipo (recoloca con ±0.7 cm
  y ±1.5° de error, descuenta 15 s, presiona A). Tras una falla crítica o una caída, reinicio en la
  zona del nivel que jugaba; la primera caída es advertencia y las siguientes restan 20.
- **Nivel 1 por comandos** (rectas con rumbo del IMU que terminan en una línea perpendicular).
- **Nivel 3 por maniobra**: salir de la zona azul, tomar el prisma con la garra, dejarlo en la
  zona punteada por distancias fijas, retroceder y cruzar la línea de la escalera para alinearse.
- **Pelota omitida** por ahora (misiones 8 y 12; pasos en `BALL_STEPS`).

## Estado (pista reconstruida)

Monte Carlo de la estrategia completa, 20 corridas con ruido desde el nivel 1: M1 a M7 100 %,
escalera 95 %, torre 90 %, colores 85 % (4 de 6 tarjetas), meta 80 %; mediana 980 puntos, sin
caídas.

## Límites

- La pista sale de una imagen: **medir** alturas, posiciones de props, muros y tarjetas (TODO en
  `arena.py`). Los colores y la luz de cada material son típicos, no calibrados.
- Las medidas del robot son estimadas (TODO en `config.py`).
- Física simplificada: el robot es un rectángulo, la escalera es una pendiente (más la regla
  de que cada oruga vaya sobre su tramo), el prisma solo se desliza en la dirección del empuje.
- El simulador informa decisiones; no prueba que el robot real lo logre.
