"""Conversión entre los marcos de referencia de MuJoCo y de ArduPilot.

Cada programa mide las direcciones a su manera:

    MuJoCo (ROS y casi toda la robótica)      ArduPilot (aviación)
    mundo ENU: x Este, y Norte, z Arriba      mundo NED: x Norte, y Este, z Abajo
    cuerpo FLU: x adelante, y izq., z arriba  cuerpo FRD: x adelante, y der., z abajo

Si se pasan los datos sin convertir, ArduPilot cree que el dron está al revés y lo voltea al despegar.
Por eso todo lo que va de MuJoCo a ArduPilot pasa por aquí (y está probado en tests/test_marcos.py).
"""
import math

import numpy as np

# ENU <-> NED: se intercambian x e y, y z cambia de signo. La matriz es su propia inversa.
ENU_NED = np.array([[0.0, 1.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0]])

# FLU <-> FRD: y y z cambian de signo. También es su propia inversa.
FLU_FRD = np.diag([1.0, -1.0, -1.0])

RADIO_TIERRA = 6378137.0  # m (WGS-84)


def enu_a_ned(v):
    """Vector del mundo (posición, velocidad) de ENU a NED."""
    return ENU_NED @ np.asarray(v)


def flu_a_frd(v):
    """Vector del cuerpo (giroscopio, acelerómetro) de FLU a FRD."""
    return FLU_FRD @ np.asarray(v)


def rotacion_ned_frd(r_enu_flu):
    """Orientación del dron: de la matriz de MuJoCo (cuerpo FLU en mundo ENU) a la de ArduPilot (FRD en NED)."""
    return ENU_NED @ np.asarray(r_enu_flu).reshape(3, 3) @ FLU_FRD


def euler_zyx(r):
    """Ángulos (alabeo, cabeceo, guiñada) en radianes de una matriz de rotación, convención aeronáutica ZYX."""
    alabeo = math.atan2(r[2, 1], r[2, 2])
    cabeceo = -math.asin(max(-1.0, min(1.0, r[2, 0])))
    guinada = math.atan2(r[1, 0], r[0, 0])
    return alabeo, cabeceo, guinada


def ned_a_geodesico(ned, lat0, lon0, alt0):
    """Posición NED (m, relativa al home) a latitud, longitud (grados) y altitud (m).

    Aproximación de Tierra plana: el error es de centímetros en un campo de unos cientos de metros.
    """
    norte, este, abajo = ned
    lat = lat0 + math.degrees(norte / RADIO_TIERRA)
    lon = lon0 + math.degrees(este / (RADIO_TIERRA * math.cos(math.radians(lat0))))
    return lat, lon, alt0 - abajo


def geodesico_a_enu(lat, lon, lat0, lon0):
    """Latitud y longitud (grados) a metros (Este, Norte) respecto al home. Es la inversa de ned_a_geodesico.

    Sirve para poner en el mundo de MuJoCo (ENU, con el home en el origen) cosas que vienen en GPS:
    los límites del campo, las áreas de búsqueda, la imagen del suelo.
    """
    este = math.radians(lon - lon0) * RADIO_TIERRA * math.cos(math.radians(lat0))
    norte = math.radians(lat - lat0) * RADIO_TIERRA
    return este, norte
