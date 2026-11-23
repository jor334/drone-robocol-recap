"""Lo que hace que el vuelo no sea "de laboratorio": viento con ráfagas, arrastre del aire y error del GPS.

El ruido de los SENSORES (IMU, barómetro, altura del GPS) lo agrega ArduPilot SITL: params/sensores_reales.parm.
Aquí va lo que ArduPilot no puede simular en modo JSON (la física la hacemos nosotros):

1. VIENTO = viento medio + turbulencia.
   - Viento medio: sopla siempre desde la misma dirección y aumenta con la altura (perfil de potencia 1/7, el
     típico sobre pasto): a 50 m sopla ~25 % más fuerte que a 10 m.
   - Turbulencia: modelo de Dryden (MIL-F-8785C, el estándar de aviación para baja altura). Son tres ráfagas
     (a lo largo del viento, de lado y vertical), cada una un ruido "con memoria": cambia suave, con un tamaño típico
     L (decenas a cientos de metros) que el dron atraviesa a su velocidad. Su fuerza σ depende del viento medio.

2. ARRASTRE: la fuerza del aire sobre el dron depende de la velocidad del dron RESPECTO AL AIRE (velocidad - viento).
   - Arrastre de forma (cuadrático): F = ½ ρ Cd·A |v| v, con un Cd·A de lado y otro de arriba/abajo.
   - Arrastre de los rotores (lineal): las hélices que empujan, al moverse de lado, frenan proporcional al empuje
     total y a la velocidad (es lo que hace que un cuadricóptero no acelere sin fin al inclinarse).
   Los Cd·A y el coeficiente de rotor son ESTIMADOS para un dron de este tamaño (no hay datos del real).

3. ERROR DEL GPS horizontal: un GPS normal (sin RTK) se equivoca 1-2 m, y ese error cambia lento (minutos).
   Se modela como un ruido con memoria (Gauss-Markov) que se suma a la posición que se le manda a ArduPilot.
   La "verdad" (la que se publica en Foxglove) no lo lleva.
"""
import math
import random

import numpy as np

RHO = 1.225                 # densidad del aire (kg/m³)
CDA_LADO = 0.10             # m²: caja + brazos + motores + hélices vistos de lado (estimado)
CDA_VERTICAL = 0.30         # m²: el dron visto desde arriba, con los discos de las hélices (estimado)
ARRASTRE_ROTOR = 0.01       # s/m: fuerza lateral = coef × empuje total × velocidad lateral (típico 0.005-0.02)

PIE = 0.3048
TURBULENCIA = {"ninguna": 0.0, "ligera": 1.0, "moderada": 2.0, "fuerte": 3.0}


class Viento:
    def __init__(self, velocidad_10m=4.0, desde_grados=180.0, turbulencia="ligera", semilla=None):
        """velocidad_10m: m/s a 10 m del suelo. desde_grados: DE DÓNDE viene (como lo dicen los meteorólogos:
        180 = viento del sur, que empuja hacia el norte)."""
        self.v10 = velocidad_10m
        hacia = math.radians(desde_grados + 180.0)
        self.direccion = np.array([math.sin(hacia), math.cos(hacia), 0.0])     # unitario, hacia donde sopla (ENU)
        self.lado = np.array([-self.direccion[1], self.direccion[0], 0.0])
        self.factor = TURBULENCIA[turbulencia]
        self.rafaga = np.zeros(3)          # u (a lo largo), v (de lado), w (vertical), en m/s
        self.rng = random.Random(semilla)

    def medio(self, altura):
        """Viento medio (m/s) a esa altura sobre el suelo."""
        return self.v10 * (max(altura, 1.0) / 10.0) ** (1 / 7)

    def paso(self, dt, altura, velocidad_aire):
        """Avanza la turbulencia dt segundos y devuelve el viento total (vector ENU, m/s) a esa altura."""
        medio = self.medio(altura)
        if self.factor > 0 and medio > 0:
            # Dryden para baja altura (fórmulas en pies, como en la norma)
            h = max(altura / PIE, 10.0)
            w20 = self.medio(20 * PIE) / PIE                    # viento a 20 ft, en ft/s
            sigma_w = 0.1 * w20 * self.factor
            sigma_uv = sigma_w / (0.177 + 0.000823 * h) ** 0.4
            l_w = h
            l_uv = h / (0.177 + 0.000823 * h) ** 1.2
            v = max(velocidad_aire / PIE, medio / PIE, 3.0)     # el dron "atraviesa" las ráfagas a esta velocidad
            for i, (sigma, largo) in enumerate(((sigma_uv, l_uv), (sigma_uv, l_uv), (sigma_w, l_w))):
                # Ruido con memoria (Ornstein-Uhlenbeck, discretizado exacto): conserva la varianza σ²
                a = math.exp(-v * dt / largo)
                self.rafaga[i] = a * self.rafaga[i] + sigma * PIE * math.sqrt(1 - a * a) * self.rng.gauss(0, 1)
        else:
            self.rafaga[:] = 0
        return (self.direccion * (medio + self.rafaga[0]) + self.lado * self.rafaga[1]
                + np.array([0.0, 0.0, self.rafaga[2]]))


def fuerza_aire(velocidad, viento, empuje_total, eje_z_dron):
    """Fuerza del aire sobre el dron (N, ENU). velocidad y viento en m/s (ENU); eje_z_dron = "arriba" del dron."""
    relativa = np.asarray(velocidad) - viento            # cómo ve el dron el aire (al revés)
    rapidez = float(np.linalg.norm(relativa))
    # Separar la velocidad relativa en la parte a lo largo del eje del dron (arriba/abajo) y la de lado
    axial = float(np.dot(relativa, eje_z_dron)) * eje_z_dron
    lateral = relativa - axial
    forma = -0.5 * RHO * rapidez * (CDA_LADO * lateral + CDA_VERTICAL * axial)
    rotores = -ARRASTRE_ROTOR * empuje_total * lateral
    return forma + rotores


class ErrorGPS:
    """Error horizontal de un GPS sin corrección: σ metros, que cambia con un tiempo típico de tau segundos."""

    def __init__(self, sigma=1.0, tau=60.0, semilla=None):
        self.sigma, self.tau = sigma, tau
        self.error = np.zeros(2)       # Este, Norte (m)
        self.rng = random.Random(semilla)

    def paso(self, dt):
        if self.sigma > 0:
            a = math.exp(-dt / self.tau)
            for i in range(2):
                self.error[i] = a * self.error[i] + self.sigma * math.sqrt(1 - a * a) * self.rng.gauss(0, 1)
        return self.error
