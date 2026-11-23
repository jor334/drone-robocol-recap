"""Reglas de SUAS 2026 que valen para cualquier campo (Team Handbook 2026, sección 3).

El reglamento mide en pies; aquí todo va en metros (1 ft = 0.3048 m).
"""
PIE = 0.3048  # m

# Alturas permitidas, sobre el suelo (AGL): entre 150 y 400 ft (3.0.4).
# Por debajo de 150 ft solo se puede estar al despegar y al aterrizar.
ALTURA_MIN_M = 150 * PIE   # 45.7 m
ALTURA_MAX_M = 400 * PIE   # 121.9 m

# Entrega de cargas (3.6): puntos si cae a menos de 50 ft del blanco
RADIO_ENTREGA_M = 50 * PIE  # 15.2 m


def dentro_de(poligono, lat, lon):
    """¿Está el punto dentro del polígono? (cuenta cuántas veces un rayo hacia el este cruza los bordes)."""
    dentro = False
    for (lat1, lon1), (lat2, lon2) in zip(poligono, poligono[1:] + poligono[:1]):
        if (lat1 > lat) != (lat2 > lat):
            lon_cruce = lon1 + (lat - lat1) * (lon2 - lon1) / (lat2 - lat1)
            if lon < lon_cruce:
                dentro = not dentro
    return dentro
