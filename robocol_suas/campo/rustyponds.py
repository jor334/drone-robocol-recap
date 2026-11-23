"""El campo donde vuela el dron: un campo de PRÁCTICA con las reglas de SUAS 2026, sobre una foto de dron real.

El terreno es un rancho del noreste de Texas (cerca de Paris, TX; a ~250 km de Tulsa y con un paisaje parecido al de
Skyway 36): pasto, árboles sueltos y en grupo, estanques, un camino de grava y construcciones.

Por qué este campo y no el oficial (Skyway 36, Tulsa): para Skyway 36 solo hay fotos públicas de 68 cm/píxel (NAIP), y
la cámara del dron ve 5.5 cm/píxel a 46 m: el suelo se veía borroso. Aquí hay una foto tomada CON DRON (DJI Mavic 3, 6 de agosto de 2025, 1.8 cm/píxel): lo que ve
la cámara simulada es una foto aérea real, con sus árboles, sombras y texturas reales. Es lo que necesita YOLO.

Foto: "RustyPonds", por CM, en OpenAerialMap (https://map.openaerialmap.org), licencia CC-BY 4.0.
Alrededor (fuera de la foto de dron): USDA NAIP (dominio público).

Lo que es de SUAS 2026 (reglas.py): alturas (46-122 m), radio de entrega, tipo de blancos. Lo inventado (diseñado
sobre la foto, ver más abajo): el home, el límite de vuelo, el área de búsqueda y dónde están los blancos.

Las fotos del suelo se descargan la primera vez (preparar(), lo llama scripts/simular.sh) en datos/texturas/.
"""
import math
import os

import numpy as np

from robocol_suas import DATOS
from robocol_suas.campo import descargas
from robocol_suas.campo.reglas import ALTURA_MAX_M, ALTURA_MIN_M, PIE, RADIO_ENTREGA_M, dentro_de  # noqa: F401
from robocol_suas.sim import marcos

NOMBRE = "rustyponds"
CREDITOS = ('Foto de dron: "RustyPonds", por CM (OpenAerialMap), CC-BY 4.0. '
            "Alrededores: USDA NAIP (dominio público).")
COG = ("https://oin-hotosm-temp.s3.us-east-1.amazonaws.com/6893c8e6e910ae14ec507257/0/"
       "6893c8e6e910ae14ec507258.tif")

# Todo el diseño se hizo en metros (Este, Norte) desde el centro de la foto de dron:
CENTRO_FOTO = (33.670998, -95.03505)


def _punto(este, norte):
    """Metros desde el centro de la foto → (lat, lon)."""
    lat, lon, _ = marcos.ned_a_geodesico([norte, este, 0], *CENTRO_FOTO, 0)
    return lat, lon


# Home: el patio de grava junto al granero. Altitud del terreno: ~165 m (estimada, de mapas topográficos de la zona).
HOME_LAT, HOME_LON = _punto(-70.0, -50.0)
HOME_ALT_MSL = 165.0
HOME = (HOME_LAT, HOME_LON, HOME_ALT_MSL, 0)


def a_enu(lat, lon):
    """Coordenada GPS a metros (Este, Norte) en el mundo de MuJoCo, que tiene el home en el origen."""
    return marcos.geodesico_a_enu(lat, lon, HOME_LAT, HOME_LON)


def desde_enu(este, norte):
    """Metros (Este, Norte) desde el home a coordenada GPS (lat, lon)."""
    lat, lon, _ = marcos.ned_a_geodesico([norte, este, 0], HOME_LAT, HOME_LON, 0)
    return lat, lon


def _enu(este_c, norte_c):
    """Metros desde el centro de la foto → metros desde el home."""
    return a_enu(*_punto(este_c, norte_c))


# Límite de vuelo: rodea todo el rancho (unos 900 × 730 m), como el de Skyway 36 (~1100 × 1000 m)
LIMITE_VUELO = [_punto(e, n) for e, n in
                [(-430, -200), (-250, -370), (420, -370), (450, 150), (320, 360), (-430, 360)]]

# Área de búsqueda: el potrero del sureste, ~9 acres (140 × 245 m; las de SUAS 2026 son de ~10 acres).
# Pasto con árboles sueltos y en grupo: los árboles pueden tapar los blancos, como pide el reglamento.
AREAS_BUSQUEDA = {1: [_punto(e, n) for e, n in [(215, -255), (355, -255), (355, -10), (215, -10)]]}

# Blancos y basura de distracción (objetos de modelos/blancos.xml), en el área de búsqueda.
# Todos sobre PASTO ABIERTO de la foto: encima de un árbol fotografiado (plano) se verían "flotando" sobre la copa.
# (Este, Norte, giro en grados), en metros desde el centro de la foto. En la competencia nadie sabe dónde están:
# el código de la misión NO debe leerlos.
_BLANCOS_FOTO = {
    "maniqui": (300.0, -150.0, 30),    # junto a un arbusto, con un carro al lado, cerca de árboles
    "carro": (295.0, -155.5, 20),
    "arbusto": (302.5, -152.0, 0),
    "tienda": (245.0, -125.0, 15),     # en un claro entre árboles
    "llanta": (265.0, -40.0, 0),
    "cono": (330.0, -110.0, 0),
    "caneca": (290.0, -30.0, 0),
    "balon": (310.0, -230.0, 0),
    "cajas": (320.0, -235.0, 25),
    "bicicleta": (300.0, -250.0, -40),
    "lona": (345.0, -60.0, 40),
}
POSICIONES_BLANCOS = {k: (*_enu(e, n), giro) for k, (e, n, giro) in _BLANCOS_FOTO.items()}
BLANCOS = {nombre: desde_enu(*POSICIONES_BLANCOS[nombre][:2]) for nombre in ("maniqui", "tienda")}

# Sol: el de la foto (6 de agosto de 2025, 21:04 UTC = 4 p. m. en Texas), calculado con las fórmulas solares de
# siempre (declinación 16.6°, ángulo horario 40°): viene del oeste-suroeste a 50° sobre el horizonte. Así las
# sombras de los objetos 3D (maniquí, carro) caen hacia el mismo lado que las de los árboles de la foto.
SOL_AZIMUT_GRADOS = 254.0
SOL_ELEVACION_GRADOS = 50.0

# ---- Suelo: tres capas, de la más grande (abajo) a la más detallada (arriba) ----
# (Este, Norte) en metros desde el centro de la foto, y metros por píxel
_CAPAS_FOTO = [
    # NAIP alrededor: 1400 × 1400 m, 0.68 m/píxel
    dict(nombre="suelo", archivo="texturas/rustyponds_naip.png", este=(-700, 700), norte=(-700, 700), m=1400 / 2048),
    # toda la foto de dron a 20 cm/píxel (donde no tiene datos, se rellena con NAIP)
    dict(nombre="suelo_foto", archivo="texturas/rustyponds_foto.png", este=(-420, 420), norte=(-360, 360), m=0.20),
    # el patio del despegue a 5 cm/píxel (100 × 100 m): para que despegue y aterrizaje también se vean nítidos
    dict(nombre="suelo_home", archivo="texturas/rustyponds_home.png", este=(-120, -20), norte=(-100, 0), m=0.05),
    # el área de búsqueda con 20-40 m de margen a 5 cm/píxel: lo que ve la cámara a 46 m (5.5 cm/píxel)
    dict(nombre="suelo_detalle", archivo="texturas/rustyponds_detalle.png", este=(195, 375), norte=(-275, 10), m=0.05),
]


def _capas():
    capas = []
    for c in _CAPAS_FOTO:
        (e0, n0), (e1, n1) = _enu(c["este"][0], c["norte"][0]), _enu(c["este"][1], c["norte"][1])
        capas.append(dict(nombre=c["nombre"], archivo=c["archivo"], este=(e0, e1), norte=(n0, n1)))
    return capas


CAPAS_SUELO = _capas()


def preparar():
    """Descarga y arma las imágenes del suelo, si faltan (solo la primera vez: ~150 MB de descargas, unos minutos)."""
    rutas = [os.path.join(DATOS, c["archivo"]) for c in _CAPAS_FOTO]
    if all(os.path.exists(r) for r in rutas):
        return
    print("Descargando el suelo del campo rustyponds (solo la primera vez, unos minutos) ...")
    os.makedirs(os.path.dirname(rutas[0]), exist_ok=True)
    hechas = []   # capas ya listas: (capa, imagen); los huecos de cada capa se rellenan con una de abajo
    for c, ruta in zip(_CAPAS_FOTO, rutas):
        if os.path.exists(ruta):            # ya estaba (una descarga anterior que se cortó después)
            hechas.append((c, descargas.leer_png(ruta)))
            continue
        sur, oeste = _punto(c["este"][0], c["norte"][0])
        norte, este = _punto(c["este"][1], c["norte"][1])
        ancho = round((c["este"][1] - c["este"][0]) / c["m"])
        alto = round((c["norte"][1] - c["norte"][0]) / c["m"])
        print(f"{c['nombre']}: {ancho} × {alto} píxeles ({c['m'] * 100:.0f} cm/píxel)")
        if not hechas:
            imagen = descargas.naip(oeste, sur, este, norte, ancho, alto)
        else:
            # la capa más detallada de abajo que la cubre entera
            debajo = [h for h in hechas if h[0]["este"][0] <= c["este"][0] and c["este"][1] <= h[0]["este"][1]
                      and h[0]["norte"][0] <= c["norte"][0] and c["norte"][1] <= h[0]["norte"][1]][-1]
            fondo = _recortar(debajo, c)
            imagen = descargas.rellenar_huecos(descargas.foto_oam(COG, oeste, sur, este, norte, ancho, alto),
                                               descargas.ampliar(fondo, ancho, alto))
        descargas.guardar_png(ruta, imagen)
        hechas.append((c, imagen))


def _recortar(anterior, c):
    """De la capa de abajo, el pedazo que queda debajo de la capa c."""
    ca, img = anterior
    alto, ancho = img.shape[:2]
    x0 = (c["este"][0] - ca["este"][0]) / (ca["este"][1] - ca["este"][0]) * ancho
    x1 = (c["este"][1] - ca["este"][0]) / (ca["este"][1] - ca["este"][0]) * ancho
    y0 = (ca["norte"][1] - c["norte"][1]) / (ca["norte"][1] - ca["norte"][0]) * alto
    y1 = (ca["norte"][1] - c["norte"][0]) / (ca["norte"][1] - ca["norte"][0]) * alto
    return img[max(0, math.floor(y0)):math.ceil(y1), max(0, math.floor(x0)):math.ceil(x1)].astype(np.uint8)
