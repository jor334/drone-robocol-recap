# El campo

Todo está en `robocol_suas/campo/rustyponds.py` (el campo) y `campo/reglas.py` (las reglas de SUAS 2026).

## Por qué un campo de práctica y no el oficial

SUAS 2026 se vuela en Skyway 36 (Tulsa). Para ese lugar solo hay fotos aéreas públicas de 68 cm/píxel (USDA NAIP), y la
cámara del dron ve 5.5 cm/píxel a 46 m: el suelo se veía como una mancha borrosa, inútil para probar la visión.

Por eso el dron vuela en un **campo de práctica con las reglas de SUAS 2026** sobre una **foto tomada con dron**:
un rancho del noreste de Texas (a ~250 km de Tulsa, paisaje parecido: pasto, árboles sueltos y en grupo, estanques,
camino de grava), fotografiado con un DJI Mavic 3 a 1.8 cm/píxel el 6 de agosto de 2025.

- Foto: "RustyPonds", por CM, en [OpenAerialMap](https://map.openaerialmap.org), **CC-BY 4.0** (se puede usar y
  publicar citando al autor). Alrededor: NAIP (dominio público).
- Se descarga sola la primera vez (`preparar()`, ~70 MB en `datos/texturas/`), recortando el GeoTIFF original con
  titiler (`campo/descargas.py`). Los huecos sin datos de la foto se rellenan con la capa de abajo.

## Capas del suelo

Planos de MuJoCo con fotos, de la más grande (abajo) a la más detallada (arriba), cada una 2 cm más arriba:

| Capa | Zona | Resolución |
|---|---|---|
| `suelo` | 1400 × 1400 m alrededor (NAIP) | 68 cm/píxel |
| `suelo_foto` | toda la foto de dron (840 × 720 m) | 20 cm/píxel |
| `suelo_home` | el patio del despegue (100 × 100 m) | 5 cm/píxel |
| `suelo_detalle` | el área de búsqueda + margen (180 × 285 m) | 5 cm/píxel (lo que ve la cámara a 46 m) |

## Lo que es de SUAS 2026 y lo que es inventado

| | Valor | De dónde |
|---|---|---|
| Alturas | entre 150 y 400 ft (46 y 122 m) sobre el suelo | reglamento (3.0.4) |
| Radio de entrega | 50 ft (15.2 m) | reglamento (3.6) |
| Blancos | un maniquí y una tienda pop-up entre basura de distracción | reglamento (3.6.2) |
| Home | patio de grava junto al granero (altitud ~165 m, estimada) | diseñado sobre la foto |
| Límite de vuelo | polígono de 6 puntos alrededor del rancho (~900 × 730 m) | diseñado sobre la foto |
| Área de búsqueda | potrero del sureste, ~9 acres (140 × 245 m) | diseñado sobre la foto |

## Blancos

`modelos/blancos.xml` (formas simples de MuJoCo): un **maniquí** acostado junto a dos arbustos y un carro, una **tienda
pop-up** abierta y basura de distracción (llanta, cono, caneca, balón, cajas, bicicleta y una lona naranja que se puede
confundir con la tienda). `POSICIONES_BLANCOS` en `rustyponds.py` dice dónde va cada uno: todos sobre **pasto abierto**,
porque encima de un árbol de la foto (que es plano) se verían flotando sobre la copa.

Las coordenadas GPS del maniquí y la tienda quedan en `campo.BLANCOS` (para medir a qué distancia cae cada carga).
**La misión no debe leerlas**: en la competencia hay que encontrarlos con la cámara.
`python3 -m robocol_suas.mision.ver_blancos` sí las usa, pero solo para revisar la simulación: vuela a 50 m sobre cada uno.

Con la cámara de 1280×960, el maniquí (1.8 m) mide ~32 px a 46 m, ~21 px a 70 m y ~15 px a 100 m.

## Geocerca

ArduPilot vigila el campo por su cuenta, aunque falle el código de la misión (`params/geocerca.parm`):

| Qué | Valor | Nota |
|---|---|---|
| Polígono | límite de vuelo del campo | lo sube `campo/geocerca.py` por MAVLink; `simular.sh` lo hace solo al arrancar (puerto 5763) |
| Techo | 120 m | 400 ft = 121.9 m |
| Piso | 47 m | se activa solo al pasar de 49 m y ArduCopter lo ignora mientras aterriza: se puede despegar y aterrizar |
| Al salirse | RTL a 50 m | vuelve al home y aterriza |

Comprobado en SITL:
- **Destino fuera del límite:** en GUIDED, ArduCopter **rechaza** un `simple_goto` a un punto fuera del polígono.
- **Salida forzada:** empujado con órdenes de velocidad, ArduPilot detectó la salida, pasó solo a RTL, desactivó el piso
  para bajar y aterrizó en el home.

Para una Pixhawk real: cargar `params/geocerca.parm` (con QGroundControl) y
`python3 -m robocol_suas.campo.geocerca --conexion /dev/ttyACM0`.
