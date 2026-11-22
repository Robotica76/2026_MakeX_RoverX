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
```

Solo usa la biblioteca estándar de Python. Cada corrida escribe `build/sim/rover_sim.html`: una
reproducción con la pista, el robot, sus sondas, la garra, el rastro, los eventos y la tabla
**juez vs. programa**. Clic en un evento o en una misión lleva a ese momento; barra
espaciadora = reproducir.

## Cómo funciona

| Archivo | Qué hace |
| --- | --- |
| [arena.py](arena.py) | Pista reconstruida de la vista superior del reglamento (1 px = 0.235 cm): líneas, zonas, marcas, alturas de niveles, rampas, escalera de dos tramos, muros del pasillo, props y tarjetas |
| [physics.py](physics.py) | Orugas (zona muerta, retardo, patinaje, derrape al girar, carga en pendiente), encoders, giroscopio, pitch por la altura del piso, Quad RGB (luz, color y RGB por sonda), brazo y garra, objetos, empujar el prisma, choques y caídas |
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

## Estado (pista reconstruida, sin ruido)

Misiones 1 a 6 pasan con el juez. **Pendientes**: el prisma se sale de la línea en la primera
curva de la S y no llega a la zona punteada (misión 7); en la escalera una oruga queda fuera de
su tramo (misión 9: revisar `robot_w` / `belt_w` contra la separación de 12 cm y la alineación
antes de subir); torre, tarjetas y meta (10 a 13) aún no se afinan. Detrás de la torre caben
pocas tarjetas antes de volver a la línea: revisar dónde van en la pista real.

## Límites

- La pista sale de una imagen: **medir** alturas, posiciones de props, muros y tarjetas (TODO en
  `arena.py`). Los colores y la luz de cada material son típicos, no calibrados.
- Las medidas del robot son estimadas (TODO en `config.py`).
- Física simplificada: el robot es un rectángulo, la escalera es una pendiente (más la regla
  de que cada oruga vaya sobre su tramo), el prisma solo se desliza en la dirección del empuje.
- El simulador informa decisiones; no prueba que el robot real lo logre.
