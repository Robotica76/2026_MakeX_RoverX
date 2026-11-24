# Control manual (Bluetooth Controller de Makeblock)

Programa para el robot: [`mblock/rover_x_manual.py`](../mblock/rover_x_manual.py) (fuente:
[`software/robot/rover_x_manual.py`](../software/robot/rover_x_manual.py) y
[`software/manual/manual.py`](../software/manual/manual.py)). En el CyberPi: **B** = empezar el
control, **A** = parar (el robot se detiene y vuelve a esperar B).

Misiones manuales (alianza): cubos M-A-K-E-X en la barra del nivel 2 (en orden y de frente),
activar las rampas (el primer acceso es por la **escalera**) y 4 cilindros (5 × 15 cm) parados
en el área negra.

| Control | Acción |
| --- | --- |
| Joystick izquierdo ↕ (`Ly`) | Avanzar / retroceder (el giroscopio sostiene el rumbo mientras no se gira) |
| Joystick derecho ↔ (`Rx`) | Girar |
| Joystick derecho ↕ (`Ry`) | Brazo fino |
| Cruceta `Up Down Left Right` | Movimientos finos lentos (con los joysticks en el centro) |
| `L1` / `L2` (mantener) | Brazo arriba / abajo (servo 0..90°, 60°/s) |
| `R1` / `R2` (mantener) | Garra abrir / cerrar (de `claw_open` a `claw_cube` − 20°; sigue la calibración aunque el servo gire al revés) |
| `N1` | Modo rápido (LEDs azules) / preciso (LEDs naranjas) |
| `N2` | Brazo a la altura de la barra (cubos MakeX) |
| `N3` (mantener) | Escalera: recto con el giroscopio a la potencia de escalera |
| `N4` | Brazo arriba (servo 90°, garra sobre la nariz): viajar y girar cerca de cosas |
| `A` en el CyberPi | Salir al menú |

- El brazo se mueve por ángulo del servo con la tabla medida (`arm_h*`/`arm_x*` en
  config.py): a 0° la garra queda baja y al frente (~8.5 cm, no llega al piso), a 90° arriba.
- `hal.drive` limita la aceleración mientras el robot va inclinado (rampas, escalera).
- Los joysticks tienen zona muerta (`PAD_DEADZONE`) y curva expo (`PAD_EXPO`).
- En la pantalla del CyberPi: modo y ángulo/altura del brazo.

## Practicar en el simulador

En el simulador web (GitHub Pages) o en tu computadora, pestaña **Manual · control**:

```bash
.venv/bin/python simulation/live.py --manual
```

Usa la **arena manual del reglamento** (`simulation/manual_arena.py`), con la vista superior y la 3D
lado a lado:

- **Medidas del reglamento:** área de 2440 × 1220 mm, nivel 2 a 20 cm, escaleras dobles de
  8 × 2.5 cm, rampas de 23 cm de ancho y unos 33°, pedestales de 6 × 6 × 10 cm, cubos de 5 cm y
  cilindros de 5 × 15 cm.
- **Medidas del dibujo:** las posiciones, el círculo negro y la barra MakeX.
- **Rampas:** empiezan levantadas. Desde el nivel 2, empújalas hacia afuera para tumbarlas
  (30 pts c/u y +40 por las dos).
- **Cubos M-A-K-E-X:** tómalos de los pedestales y suéltalos en el lugar de su letra en la barra
  (50 c/u). El bono de 100 pide el orden correcto con la letra mirando al frente; la rayita en la
  vista superior indica hacia dónde mira la letra.
- **Cilindros:** déjalos parados y completamente dentro del área negra (50 c/u).
- **Física** (`simulation/manual_sim.py`):
  - el robot empuja los objetos;
  - un cilindro se voltea si lo golpeas alto (con la garra) o rápido;
  - los objetos se caen de los niveles;
  - la garra solo toma un objeto que esté entre los dedos, a su altura, y solo cuando cierra lo
    suficiente;
  - al soltar desde más de 2.5 cm, el cilindro se cae.
- **Colocar en la zona:** regresa el robot a la X roja.

En la página, **Control Bluetooth de Makeblock (emulado)**: la misma distribución del control
real (como el emulador del repo Challengue): clic en los botones, arrastrar los joysticks, On/Off.
Teclado: W/S joystick izq. ↕ · J/L joystick der. ↔ · I/K joystick der. ↕ · flechas = cruceta ·
Q/E = L1/L2 · U/O = R1/R2 · 1-4 = botones 1-4 · Mayús = joysticks a la mitad. Un control real
conectado a la PC (USB/Bluetooth) también funciona.
