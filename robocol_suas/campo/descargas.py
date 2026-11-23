"""Descarga de imágenes aéreas para el suelo de MuJoCo.

Dos fuentes, las dos pedidas en latitud/longitud (EPSG:4326): así cada píxel es un paso fijo en grados, igual que
la Tierra plana local de sim/marcos.py, y la imagen encaja en metros sobre un plano de MuJoCo.

- NAIP (USDA, servida por USGS The National Map): todo EE. UU., ~0.6 m/píxel, dominio público.
- OpenAerialMap (OAM): fotos de dron que la gente comparte, de 1 a 5 cm/píxel, casi todas CC-BY 4.0 (se puede usar
  y publicar citando al autor). Se recortan con titiler (titiler.hotosm.org), que lee el GeoTIFF original por partes.

MuJoCo solo lee texturas PNG: todo se guarda en PNG.
"""
import time
import urllib.parse
import urllib.request

import cv2
import numpy as np

NAIP = "https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPImagery/ImageServer/exportImage"
TITILER = "https://titiler.hotosm.org/cog/bbox"
MAX_LADO = 1000  # titiler falla (error 500) con pedidos grandes, sobre todo a resolución alta: pedazos de 1000


def _bajar(url, intentos=5):
    for intento in range(1, intentos + 1):
        try:
            with urllib.request.urlopen(url, timeout=180) as respuesta:
                return respuesta.read()
        except OSError as error:
            print(f"  error ({intento}/{intentos}): {error}")
            time.sleep(5)
    raise SystemExit(f"No se pudo descargar {url[:120]}...")


def naip(oeste, sur, este, norte, ancho, alto):
    """Foto NAIP del recuadro (grados), como imagen RGB (alto × ancho × 3, uint8)."""
    parametros = {"bbox": f"{oeste},{sur},{este},{norte}", "bboxSR": 4326, "imageSR": 4326,
                  "size": f"{ancho},{alto}", "format": "jpgpng", "f": "image"}
    datos = _bajar(NAIP + "?" + urllib.parse.urlencode(parametros))
    imagen = cv2.imdecode(np.frombuffer(datos, np.uint8), cv2.IMREAD_COLOR)
    if imagen is None:
        raise SystemExit("El servidor de NAIP no devolvió una imagen (a veces está saturado: inténtalo más tarde).")
    return cv2.cvtColor(imagen, cv2.COLOR_BGR2RGB)


def foto_oam(cog, oeste, sur, este, norte, ancho, alto):
    """Recorte de una foto de OpenAerialMap (URL de su GeoTIFF), como RGBA: alfa = 0 donde la foto no tiene datos."""
    salida = np.zeros((alto, ancho, 4), np.uint8)
    for fila0 in range(0, alto, MAX_LADO):
        for col0 in range(0, ancho, MAX_LADO):
            fila1, col1 = min(fila0 + MAX_LADO, alto), min(col0 + MAX_LADO, ancho)
            # el pedazo de recuadro (en grados) que corresponde a esas filas y columnas
            o = oeste + (este - oeste) * col0 / ancho
            e = oeste + (este - oeste) * col1 / ancho
            n = norte - (norte - sur) * fila0 / alto
            s = norte - (norte - sur) * fila1 / alto
            url = (f"{TITILER}/{o},{s},{e},{n}/{col1 - col0}x{fila1 - fila0}.png?"
                   + urllib.parse.urlencode({"url": cog, "dst_crs": "epsg:4326", "coord_crs": "epsg:4326"}))
            print(f"  pedazo {fila0 // MAX_LADO + 1}-{col0 // MAX_LADO + 1} ({col1 - col0} × {fila1 - fila0})")
            pedazo = cv2.imdecode(np.frombuffer(_bajar(url), np.uint8), cv2.IMREAD_UNCHANGED)
            if pedazo.shape[2] == 3:
                pedazo = np.dstack([pedazo, np.full(pedazo.shape[:2], 255, np.uint8)])
            salida[fila0:fila1, col0:col1] = cv2.cvtColor(pedazo, cv2.COLOR_BGRA2RGBA)
    return salida


def rellenar_huecos(rgba, fondo_rgb, borde_px=6):
    """Donde la foto no tiene datos (alfa = 0) pone el fondo, con un borde suave para que no se note el corte."""
    alfa = (rgba[..., 3] > 0).astype(np.float32)
    alfa = cv2.erode(alfa, np.ones((3, 3), np.uint8), iterations=2)          # los bordes de la foto suelen ser feos
    alfa = cv2.GaussianBlur(alfa, (0, 0), borde_px)[..., None]
    mezcla = rgba[..., :3].astype(np.float32) * alfa + fondo_rgb.astype(np.float32) * (1 - alfa)
    return np.clip(mezcla, 0, 255).astype(np.uint8)


def guardar_png(ruta, rgb):
    cv2.imwrite(ruta, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_PNG_COMPRESSION, 3])


def leer_png(ruta):
    return cv2.cvtColor(cv2.imread(ruta), cv2.COLOR_BGR2RGB)


def ampliar(rgb, ancho, alto):
    return cv2.resize(rgb, (ancho, alto), interpolation=cv2.INTER_CUBIC)


__all__ = ["naip", "foto_oam", "rellenar_huecos", "guardar_png", "leer_png", "ampliar"]
