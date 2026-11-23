# Cómo funciona la simulación

## El puente ArduPilot ↔ MuJoCo (`sim/puente_sitl.py`)

ArduPilot SITL tiene un modo llamado **JSON** en el que no calcula la física él mismo: en cada uno de sus ciclos
(1200 por segundo) manda el PWM de los motores a un programa externo y espera a que le responda dónde quedó el dron.
Ese programa es el puente:

1. recibe el PWM de los 4 motores por UDP (puerto 9002),
2. lo convierte en empuje (curva de `MOT_THST_EXPO`, igual que ArduPilot),
3. suma la fuerza del viento (`sim/perturbaciones.py`) y avanza la física un paso (`mj_step`),
4. contesta con IMU, posición, velocidad y actitud en JSON.

Es un **ping-pong sincronizado** (*lockstep*): ArduPilot no avanza hasta recibir la respuesta, así que nunca se
desincronizan aunque el PC vaya lento (en ese caso todo va "en cámara lenta" y el puente lo avisa).

GPS, barómetro y brújula los simula ArduPilot a partir de ese estado (con ruido: `params/sensores_reales.parm`).
El controlador que estabiliza el dron **no es nuestro**: es el de ArduPilot, con sus modos de siempre (GUIDED, AUTO,
LAND, RTL…).

## Marcos de referencia (la trampa clásica)

MuJoCo mide en **ENU** (x Este, y Norte, z Arriba) y el cuerpo en **FLU** (adelante, izquierda, arriba). ArduPilot usa
**NED** (Norte, Este, Abajo) y **FRD** (adelante, derecha, abajo). Si se pasan los datos sin convertir, ArduPilot cree que
el dron está al revés y lo voltea al despegar. La conversión está en `sim/marcos.py` y tiene pruebas en
`tests/test_marcos.py` (por ejemplo: quieto en el suelo, el acelerómetro debe marcar `(0, 0, -9.81)` en FRD).

El origen del mundo de MuJoCo es el **home** (el punto de despegue). `campo.a_enu(lat, lon)` pasa de GPS a metros.

## La escena de MuJoCo

- `modelos/dron.xml`: el dron (escrito a mano; ver [dron.md](dron.md)).
- `modelos/blancos.xml`: plantilla del maniquí, la tienda y la basura de distracción.
- `campo/escena.py` arma **`datos/escena.xml`**: incluye el dron y le suma el campo (fotos del suelo, sol, blancos en su
  posición). Se vuelve a escribir cada vez que se carga el modelo con `robocol_suas.modelos.cargar()`. No se edita a mano.

## La cámara (`sim/camara.py`, `viz/foxglove_pub.py`)

Cámara de 1280×960 mirando hacia abajo, con OSMesa (renderizado por software: en Docker no hay tarjeta gráfica). Cada foto
tarda 200-350 ms, así que corre en **su propio hilo**: la física no la espera y, si una foto no está lista, ese cuadro
se salta (salen ~3-6 por segundo). Cada foto va a Foxglove (`/camara/imagen`) y a `vision.py` (`Vision.process_frame`).

## Foxglove (`viz/foxglove_pub.py`)

Foxglove se conecta sin ROS, con el SDK de Python (puerto 8765). Todo lo que se publica es la **verdad del simulador**;
lo que estima ArduPilot se lee con DroneKit (`dron.location`, `dron.attitude`…).

| Tópico | Qué | Panel |
|---|---|---|
| `/tf` | posición y orientación del dron y de la cámara | 3D |
| `/escena/dron`, `/escena/blancos` | figuras del dron y de los blancos (sacadas de los XML) | 3D |
| `/escena/campo`, `/campo` | límite de vuelo (rojo) y área de búsqueda (amarillo); en el mapa, también los blancos | 3D, Map |
| `/escena/trayectoria` | camino recorrido | 3D |
| `/gps` | lat/lon del dron | Map |
| `/camara/imagen`, `/camara/calibracion` | la foto (JPEG) y los parámetros de la cámara (para pasar de píxel a suelo) | Image |
| `/estado` | PWM, empuje de cada motor, altura, actitud, viento, error del GPS | Plot |

Si el puerto 8765 ya está ocupado (por ejemplo, un `foxglove_bridge` de ROS): `FOXGLOVE_PUERTO=8766 docker compose up -d`
y la dirección pasa a ser `ws://localhost:8766`.

**Estación de tierra (opcional):** QGroundControl o Mission Planner se conectan por TCP a `localhost:5762`.

## Puertos

| Puerto | Qué |
|---|---|
| UDP 9002 | ArduPilot ↔ puente (física) |
| TCP 5760 | DroneKit (misiones) |
| TCP 5762 | QGroundControl, desde fuera del contenedor |
| TCP 5763 | subida de la geocerca al arrancar |
| 8765 | Foxglove y visor 3D |

## Problemas comunes

- **`bind failed on port 5760`** al arrancar: quedó un ArduPilot colgado (SITL ignora la señal normal de terminar):
  `pkill -9 -x arducopter`.
- **ArduPilot repite `No JSON sensor message received`**: el puente no le contesta. Para ver dónde se quedó, en otra
  terminal del contenedor: `pkill -USR1 -f puente_sitl` (imprime la pila de cada hilo).
- **`[puente] Simulación más lenta que el tiempo real: x0.56`**: normal en los primeros segundos (arranque y carga de las
  fotos). Si sigue, probar `--resolucion 640x480` o `--sin-camara`.
