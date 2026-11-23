"""La cámara del dron: sacar una foto de MuJoCo que se parezca a una foto aérea de verdad.

Dos cosas que MuJoCo no hace solo:

1. Sombras en todo el campo. MuJoCo calcula las sombras de una luz direccional (el sol) en un cuadrado fijo
   alrededor de la posición de la luz (±70 m, ver <map shadowclip> en dron.xml). Si el sol se quedara
   quieto sobre el home, el maniquí a 330 m no tendría sombra. Por eso, antes de cada foto, el sol se pone
   encima del punto del suelo que mira la cámara: la dirección de la luz no cambia, solo desde dónde sale.

2. Defectos de una cámara real (ver efectos_de_camara): ruido del sensor, viñeteo (bordes más oscuros) y un poco
   de desenfoque. Un YOLO entrenado con fotos reales se acostumbró a esos defectos; las imágenes "perfectas" de
   un simulador lo confunden más de lo que parece.
"""
import math

import cv2
import numpy as np

from robocol_suas.campo import rustyponds as campo


def hacia_el_sol():
    """Vector unitario hacia el sol (Este, Norte, Arriba), el mismo de dron.xml."""
    az, el = math.radians(campo.SOL_AZIMUT_GRADOS), math.radians(campo.SOL_ELEVACION_GRADOS)
    return np.array([math.sin(az) * math.cos(el), math.cos(az) * math.cos(el), math.sin(el)])


def poner_sol_encima(escena, punto_suelo, distancia=200.0):
    """Mueve las luces que hacen sombra (el sol) para que su cuadrado de sombras quede centrado en punto_suelo."""
    s = hacia_el_sol()
    for i in range(escena.nlight):
        luz = escena.lights[i]
        if luz.castshadow and not luz.headlight:
            luz.pos[:] = np.asarray(punto_suelo) + distancia * s
            luz.dir[:] = -s


class EfectosDeCamara:
    """Lo que le pasa a una foto en una cámara de verdad (valores moderados, de una cámara buena a 1280×960)."""

    def __init__(self, ancho, alto, ruido=3.0, vineteo=0.15, desenfoque=0.6, semilla=0):
        self.ruido = ruido                  # desviación del ruido del sensor (en niveles de 0-255)
        self.desenfoque = desenfoque        # sigma (píxeles): lente + vibración del dron
        # Viñeteo: la luz cae hacia los bordes (ley cos⁴ de las lentes, aproximada)
        y, x = np.mgrid[0:alto, 0:ancho].astype(np.float32)
        r2 = ((x - ancho / 2) ** 2 + (y - alto / 2) ** 2) / ((ancho / 2) ** 2 + (alto / 2) ** 2)
        self.ganancia = (1 - vineteo * r2)[..., None]
        # Generar ruido al azar para cada foto tarda ~15 ms: se generan 8 imágenes de ruido y se van rotando
        rng = np.random.default_rng(semilla)
        self.ruidos = [rng.normal(0, ruido, (alto, ancho, 1)).astype(np.float32) for _ in range(8)] if ruido > 0 else []
        self.cual = 0

    def aplicar(self, rgb):
        img = rgb.astype(np.float32)
        if self.desenfoque > 0:
            img = cv2.GaussianBlur(img, (0, 0), self.desenfoque)
        img *= self.ganancia
        if self.ruidos:
            img += self.ruidos[self.cual]                                   # ruido de brillo
            self.cual = (self.cual + 1) % len(self.ruidos)
        return np.clip(img, 0, 255).astype(np.uint8)


def fotografiar(renderizador, datos, camara, efectos=None):
    """Foto de la cámara con el sol (y sus sombras) encima del punto del suelo que se está mirando."""
    renderizador.update_scene(datos, camera=camara)
    id_camara = camara if isinstance(camara, int) else renderizador.model.camera(camara).id
    pos = datos.cam_xpos[id_camara]
    adelante = -datos.cam_xmat[id_camara].reshape(3, 3)[:, 2]     # la cámara mira hacia su -z
    if adelante[2] < -0.1:   # mirando hacia abajo: el punto donde el centro de la imagen toca el suelo
        punto = pos + adelante * (pos[2] / -adelante[2])
    else:
        punto = pos
    poner_sol_encima(renderizador.scene, [punto[0], punto[1], 0.0])
    rgb = renderizador.render()
    return efectos.aplicar(rgb) if efectos is not None else rgb
