# Realismo: cámara, viento y sensores

El objetivo es poder probar dos cosas que en el dron real no se pudieron probar (nunca voló):
**cómo se comporta en vuelo** y **qué ve la cámara** (para el modelo de visión).

## Lo que ve la cámara

1. **Suelo de foto real.** En el área de búsqueda y en el despegue, el suelo es una foto de dron a 5 cm/píxel, lo mismo
   que ve la cámara a 46 m. Árboles, sombras y texturas son reales (ver [campo.md](campo.md)).
2. **Sol con sombras.** El de la foto (6 de agosto, 4 p. m.): del oeste-suroeste a 50°. Así las sombras del maniquí y
   el carro caen hacia el mismo lado que las de los árboles de la foto. MuJoCo solo calcula sombras en un cuadrado de
   ±70 m, por eso `sim/camara.py` pone el sol encima de lo que mira la cámara antes de cada foto.
3. **Defectos de una cámara real** (`sim/camara.py`): un poco de desenfoque, ruido del sensor y bordes más oscuros
   (viñeteo). `--camara-ideal` los quita.

Lo que sigue siendo de simulador: los **blancos** son formas simples (cápsulas, cajas, elipsoides).

## Viento (lo agrega el puente: `sim/perturbaciones.py`)

ArduPilot no simula viento cuando la física es externa, así que lo hace el puente:

| Qué | Por defecto | Opción |
|---|---|---|
| Viento medio | 4 m/s a 10 m de altura, del sur; sube con la altura (perfil 1/7: ~5 m/s a 50 m) | `--viento VEL,DESDE` (`0,0` = sin viento) |
| Ráfagas | turbulencia de **Dryden** (MIL-F-8785C, la norma de aviación para baja altura), ligera | `--turbulencia ninguna/ligera/moderada/fuerte` |
| Arrastre del aire | de forma (½ρ·Cd·A·v²; Cd·A 0.10 m² de lado, 0.30 m² de arriba) + de los rotores (0.01 × empuje × velocidad lateral) | constantes en el archivo (**estimadas**) |
| Error del GPS horizontal | ~1 m, que cambia lento (~1 min), como un GPS sin RTK | `--error-gps M` |

El arrastre depende de la velocidad **respecto al aire**: con viento, el dron se inclina para quedarse quieto, como el
real. MuJoCo no calcula aire por su cuenta (`density` = 0 en `dron.xml`) para no contarlo dos veces.

## Ruido de los sensores (lo agrega ArduPilot: `params/sensores_reales.parm`)

Acelerómetros 0.6 m/s², giroscopios ~1.3 °/s en vuelo, barómetro ±0.4 m, altura del GPS ±0.8 m, GPS a 5 Hz con
100 ms de retraso.

En Foxglove, `/estado` trae el viento y el error del GPS de cada momento; `/gps` y `/tf` siguen siendo la verdad.

## Medido (2026-11-22)

40 s quieto a 50 m, con el viento por defecto: el dron se inclina **6.3° contra el viento**, las ráfagas lo mueven
**±0.6°** y la altura varía ±2 cm. Simulación en tiempo real.

## Visor 3D (`viz/visor_3d.py`)

```bash
scripts/visor.sh        # en el ordenador, FUERA de Docker (usa su tarjeta gráfica). En Mac lo abre con mjpython
```
Ventana de MuJoCo con la misma escena, a 60 cuadros por segundo, con cielo, neblina a lo lejos y hélices girando.
Solo mira: lee la posición del dron y el empuje de los motores del mismo WebSocket que Foxglove, así que funciona con la
simulación ya corriendo. Cámara de persecución al empezar; ratón para moverla, `[` `]` para cambiar de cámara,
Esc = libre, Inicio = volver a la persecución. Los árboles son los de la foto: desde arriba son reales, de lado se ven planos.

## Por qué MuJoCo y no un motor gráfico

MuJoCo es fuerte en física, no en gráficos: su renderizador es sencillo (sin iluminación física). Los proyectos "con GPU"
de MuJoCo (MJX, MuJoCo Warp, Madrona) aceleran la cantidad de simulaciones, no la calidad de imagen, y necesitan NVIDIA.
Para imágenes foto realistas se usa otro motor solo para dibujar (Unreal, Blender). Aquí el realismo de la cámara sale
de usar **fotos reales** como suelo.
