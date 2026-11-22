# Control manual (Bluetooth Controller de Makeblock)

Código: [`software/manual/manual.py`](../software/manual/manual.py).

| Control | Acción |
| --- | --- |
| Joystick izquierdo ↕ (`Ly`) | Avanzar / retroceder |
| Joystick derecho ↔ (`Rx`) | Girar |
| Cruceta `Up Down Left Right` | Movimientos finos lentos (si no se mueven los joysticks) |
| `N1` | Alterna modo rápido (LEDs azules) / preciso (LEDs naranjas) |
| `N3` (mantener) | Potencia máxima recta para la escalera |
| `L1` / `L2` | Brazo arriba / abajo |
| `R1` / `R2` | Garra abrir / cerrar |
| `N2` | Brazo a la altura de la barra (cubos MakeX) |
| `N4` | Brazo a la posición de viaje |
| `B` en el CyberPi | Salir al menú |

Los joysticks tienen zona muerta (`PAD_DEADZONE`) y curva expo (`PAD_EXPO`) para mover con
precisión cerca del centro.
