# calibrations en bloques (mBlock)

Versión en bloques de [calibrations.py](../software/robot/calibrations.py), con los bloques
propios del mBot2 en mBlock. Sirve para **comparar**: los bloques de mBlock ("avanzar X cm",
"girar X°") usan su propio control; el programa en Python usa nuestras medidas (rueda 4.18 cm,
orugas 0.977, giroscopio con freno). Si los dos hacen lo mismo en el piso, las medidas están bien.

> El archivo [calibrations.mblock](../software/robot/calibrations.mblock) abre en mBlock el
> programa en **Python** (como el del compañero). Para tener la versión en **bloques** como
> archivo, arma en mBlock un programa chiquito con los bloques de motores del mBot2 (abajo),
> guárdalo y mándalo: con esos bloques de ejemplo se puede generar el archivo completo.

## Secuencia (botón A)

```
cuando se presiona el botón [A]
├─ restablecer ángulo de rotación [todos]
├─ avanzar a [50] RPM durante [20] cm                    ← 1. avanza 20 cm
├─ esperar [0.5] s
├─ girar a la izquierda [90] °                           ← 2. gira 90° izquierda
├─ esperar [0.5] s
├─ avanzar a [50] RPM durante [10] cm                    ← 3. avanza 10 cm
├─ esperar [0.5] s
├─ girar a la derecha [90] °                             ← 4. gira 90° derecha
├─ esperar [0.5] s
├─ girar a la izquierda [90] °                           ← 5. gira 90° izquierda
├─ esperar [0.5] s
├─ avanzar a [50] RPM durante [20] cm                    ← 6. avanza 20 cm
├─ esperar [0.5] s
├─ restablecer ángulo de rotación [todos]               ← 7. círculo de 40 cm (izquierda)
├─ establecer velocidad del motor encoder [EM1] a [16] RPM
├─ establecer velocidad del motor encoder [EM2] a [-64] RPM   (EM2 va al revés: negativo = adelante)
├─ repetir hasta que < valor absoluto de (ángulo de rotación [z]) > [359] >
│     (nada)
├─ detener motor encoder [todos]
└─ mostrar etiqueta (ángulo de rotación [z]) …            ← cuánto marcó el giroscopio
```

**Velocidades del círculo**: diámetro 40 → radio 20 cm. Las orugas están a 18.35 cm (centro a
centro) y derrapan al girar (SKID ≈ 1.3, falta calibrar): medio ancho efectivo
18.35 × 1.3 / 2 = 11.9 cm. Oruga de adentro: 40 × (20 − 11.9)/20 = **16 RPM**; de afuera:
40 × (20 + 11.9)/20 = **64 RPM**. Si el círculo sale más chico que 40 cm, SKID es menor (bajar
la diferencia); si sale más grande, SKID es mayor.

## Prueba a mano del giroscopio

```
cuando se presiona el botón [B]
├─ restablecer ángulo de rotación [todos]
└─ por siempre
      mostrar etiqueta (ángulo de rotación [z]) en [centro] tamaño [grande]
```

Poner el robot contra una escuadra, presionar B, girarlo **a mano** una vuelta completa hasta la
misma escuadra y leer la pantalla. `gyro_scale = 360 / lectura`. Si marca ~277, el giroscopio
mide 30 % de menos (el "× 1.3" del programa del compañero) y hay que poner `gyro_scale = 1.3`.

## Qué anotar (y dónde va en config.py)

| Prueba | Medir en el piso | Ajuste |
| --- | --- | --- |
| Avanza 20 / 10 / 20 | distancia real | `sprocket_d` × real / pedido |
| Recto | ¿se desvía siempre al mismo lado? | `lr_ratio` |
| Giros 90° | ángulo real (transportador o escuadra) | si la pantalla dice 90 y el piso no: `gyro_scale` |
| Círculo 40 cm | diámetro real del rastro | `skid` |
| A mano 360° | lectura del giroscopio | `gyro_scale = 360 / lectura` |
