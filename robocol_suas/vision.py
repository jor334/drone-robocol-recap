"""Punto de entrada para recibir y analizar frames de la cámara del dron.

``Vision.process_frame`` recibe cada imagen RGB que el simulador publica en
Foxglove como ``/camara/imagen``. Sustituye o amplía ese método para agregar
detección de objetos, segmentación u otro procesamiento.
"""


class Vision:
    def process_frame(self, frame_rgb):
        """Recibe un frame RGB como arreglo NumPy; sobreescribir para analizarlo."""
        height, width, _ = frame_rgb.shape
        return {"width": width, "height": height}
