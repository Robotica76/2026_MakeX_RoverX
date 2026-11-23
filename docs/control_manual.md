# Control manual (Bluetooth Controller de Makeblock)

Código: [`software/manual/manual.py`](../software/manual/manual.py). Se entra desde la pantalla
de inicio del programa de competencia con **A**; se sale con **A** (abortar).

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
| `R1` / `R2` (mantener) | Garra abrir / cerrar |
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

```bash
.venv/bin/python simulation/live.py --manual --level 2
```

En la página, **Control Bluetooth de Makeblock (emulado)**: la misma distribución del control
real (como el emulador del repo Challengue): clic en los botones, arrastrar los joysticks, On/Off.
Teclado: W/S joystick izq. ↕ · J/L joystick der. ↔ · I/K joystick der. ↕ · flechas = cruceta ·
Q/E = L1/L2 · U/O = R1/R2 · 1-4 = botones 1-4 · Mayús = joysticks a la mitad. Un control real
conectado a la PC (USB/Bluetooth) también funciona.
