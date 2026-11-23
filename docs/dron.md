# El dron en MuJoCo (`robocol_suas/modelos/dron.xml`)

Reconstruido a partir de las **fotos del dron real y de su CAD** (Inventor):

- Quad en X de **1000 mm de motor a motor** (dato del equipo).
- 4 brazos de tubo de carbono que salen por el centro de las caras de una caja central de dos pisos (girada 45°
  respecto al frente).
- Soportes cuadrados con motor y hélice bipala al final de cada brazo.
- Sin patas: se apoya en la placa de abajo. El CAD tiene una propuesta de patas tipo patín que el dron de la foto no lleva.
- Motores **T-Motor MN4014-11 KV330** (sale de los archivos del motor en el CAD).
- Cámara mirando hacia abajo, debajo de la caja.

## Lo estimado (falta confirmarlo con el equipo)

| Dato | Valor en el modelo | De dónde sale |
|---|---|---|
| Masa total | 4.54 kg | piezas típicas: estructura de carbono, motores de 171 g (dato de T-Motor), batería 6S ~1.3 kg, electrónica, cargas de SUAS |
| Hélices | 16x5.4 | proporciones de la foto |
| Empuje máximo | 25.5 N por motor | tabla de T-Motor del MN4014 KV330 con 16x5.4 en 6S (2600 g); con 15x5 serían 18.8 N |
| Par de reacción | 0.022 m × empuje | potencia y rpm de la tabla de T-Motor (16x5.4) |
| Caja, tubos | 22 × 22 cm, tubo de 25 mm | proporciones de la foto |
| Arrastre | Cd·A 0.10 / 0.30 m², rotor 0.01 | valores típicos de un dron de este tamaño (`sim/perturbaciones.py`) |

Con los datos reales (motores, hélices, batería, peso en báscula) se cambian `mass` y `ctrlrange` en `dron.xml` y
`MOT_THST_HOVER` en `params/robocol_quad.parm` (= peso en N / empuje máximo total).

## Ajuste del control de guiñada

Con los valores por defecto de ArduPilot (pensados para drones pequeños) el dron quedaba oscilando al girar, con los
motores subiendo y bajando de 5 a 20 N durante 20 s: pedían más aceleración de giro (270°/s²) de la que este dron de
1 m y 4.5 kg puede dar. En `params/robocol_quad.parm`: `ATC_ACCEL_Y_MAX 9000`, `ATC_RAT_YAW_P 1.0`,
`ATC_RAT_YAW_I 0.1`, `ATC_ANG_YAW_P 3.0`. Con eso se queda quieto. En un dron real se afinaría con AutoTune.

## Probado

- `despegue_basico`: arma, despega a 10 m, avanza 20 m, gira 180°, vuelve al inicio, aterriza y desarma.
- Sube 60 m en 28 s; quieto sin viento se mantiene nivelado a ±0.03°; aterriza a 0.0-0.1 m del punto pedido.

## Para volar un dron real

Lo único que cambia es la conexión de DroneKit:
`python3 -m robocol_suas.mision.despegue_basico --conexion /dev/ttyACM0 --baud 115200` (USB) o el puerto de la telemetría.
