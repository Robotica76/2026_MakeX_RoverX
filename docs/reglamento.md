# Reglamento Rover X: resumen para el software

Fuente: [Reglamento Rover X (CreativaKids)](../resources/reglamento/Reglamento_Rover_X.pdf).
Vistas de la arena: [resources/reglamento/arena](../resources/reglamento/arena/).

## Formato

- 1 o 2 participantes (11 a 18 años) y un mentor. Al menos 3 rondas de clasificación (los
  puntos se suman); clasifica la mitad. La final tiene 3 rondas.
- **5 minutos de reloj** por ronda. En empate gana el menor tiempo y, después, el menor
  número de faltas.
- Robot: **mBot2 con kit Rover** (sin baterías externas) o mBot Ranger (7.4 V, 3000 mA). Se
  permiten piezas y sensores Makeblock sin límite, más piezas propias impresas, cortadas o
  de cartón.
- **Primera parte autónoma** (sin control inalámbrico ni alámbrico). **Segunda parte manual**
  con el *Bluetooth Controller* de Makeblock, en alianza con otro equipo.
- El robot debe tolerar **cambios de luz y ruido**; no se pueden pedir condiciones especiales.
- Entre rondas se puede cambiar el programa.

## Pista (2440 × 2440 mm)

Línea negra de 20 mm (±50%) sobre blanco, de unos 10 m de largo. Rampas de 22 cm de ancho
mínimo y hasta 40° de inclinación. Huecos de hasta 10 mm entre tableros. Marcas de color
como referencia. Zonas de inicio por nivel: **roja (1), amarilla (2), azul (3), verde (4)**.
Meta: **X azul** en el nivel 4.

| Prop | Medidas |
| --- | --- |
| Pedestal | 6 × 6 cm, 10 cm de alto |
| Cubo | ~50 mm, impresión 3D |
| Pelota | 3 cm de diámetro |
| Obstáculos (recortes) | ≤ 75 × 75 × 10 mm |
| Prisma | 15 cm de alto |
| Escalera | 8 peldaños de 2.5 cm, huella de 3.7 cm, dos tramos separados 12 cm |
| Torre | 10 × 10 cm, 25 cm de alto, fija |
| Barra de pelotas | 6 posiciones con agujeros de 25 mm, 10 cm de alto |

## Misiones autónomas (máx. 1270, más 100 de bono de alianza)

| # | Misión | Pts | Condición de validez |
| --- | --- | --- | --- |
| 1 | Seguir la línea, pasar el tope, parar frente al pedestal | 50 | Parar en la zona frente al pedestal |
| 2 | Tomar el cubo | 50 | Sin contacto con la arena ni el pedestal ≥ 5 s, a ≥ 5 cm de la base |
| 3 | Subir al nivel 2 (rampa) | 50 | Completamente dentro |
| 4 | Dejar el cubo en el pedestal | 100 | No se cae y no toca al robot |
| 5 | Cruzar la zona de obstáculos | 100 | Salir totalmente |
| 6 | Subir al nivel 3 (rampa) | 100 | Completamente dentro |
| 7 | Empujar el prisma a la zona punteada | 100 | Al menos parcialmente dentro |
| 8 | Tomar la pelota | 100 | Igual que el cubo |
| 9 | Subir al nivel 4 (escalera) | 150 | Completamente dentro |
| 10 | Rodear la torre | 100 | Rebasarla por completo |
| 11 | Reconocer 6 tarjetas de color | 20 c/u | Mostrar el color en LEDs o pantalla |
| 12 | Pelota en la casilla de la tarjeta **roja** | 150 | Bien puesta |
| Extra | Parar solo dentro de la zona azul | 100 | Completamente dentro; +100 si los dos equipos lo logran |

## Misiones manuales (alianza, los puntos son para ambos equipos)

| Misión | Pts |
| --- | --- |
| Cubos M-A-K-E-X en la barra del nivel 2 | 50 por letra, +100 si queda "MakeX" en orden y de frente |
| Activar las rampas (el primer acceso es por la escalera) | 30 por rampa, +40 si son las dos |
| 4 cilindros (5 × 15 cm) parados en el área negra | 50 c/u |

## Reglas que afectan al programa

- **No arrancar antes de la señal del juez**: el programa espera el botón A.
- Penalizaciones (la primera falta es advertencia, las siguientes restan 20): salirse de la
  arena o **caerse de un nivel**, tocar el robot sin permiso.
- **Reinicio**: se pueden pedir los que se quieran. En la arena automática se reinicia desde
  la **zona de color del último nivel completado** o desde el inicio del nivel 1. Por eso el
  programa detecta el color de la zona y permite elegir el nivel.
- Las tareas que no se avisan al juez no cuentan.
- Si el robot pierde una pieza no hay pausa ni repetición.

## Preguntas abiertas (preguntar a makex@creativakids.com o en la junta de capitanes)

1. ¿Los 5 minutos se reparten entre autónomo y manual, o cada parte tiene su propio reloj?
   Ahora `AUTO_BUDGET_S = 300`.
2. ¿Cuáles son los 6 colores de las tarjetas y dónde están exactamente (sobre la línea o al lado)?
3. ¿Cuáles son las medidas de la zona punteada del prisma y su posición respecto a las marcas rojas?
4. ¿Las marcas verdes del nivel 1 indican giros (estilo WRO) o son solo referencia?
