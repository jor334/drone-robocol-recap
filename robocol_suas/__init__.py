"""Robocol SUAS 2026: recreación en simulación del cuadricóptero de Robocol."""
import os

# Carpeta de lo que se descarga o se genera (fotos del suelo, escena de MuJoCo, logs de ArduPilot). No va al repo.
DATOS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "datos")
