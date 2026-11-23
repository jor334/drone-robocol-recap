"""Pruebas del campo (campo/rustyponds.py): que límites, área de búsqueda, suelo, blancos y la escena de MuJoCo
encajen entre sí.

    python3 -m pytest tests

Las pruebas que cargan la escena necesitan las fotos del suelo (no están en el repo): si faltan, se saltan.
Se descargan con scripts/simular.sh o con: python3 -c 'from robocol_suas.campo import rustyponds; rustyponds.preparar()'
"""
import math
import os

import mujoco
import numpy as np
import pytest

from robocol_suas import DATOS, modelos
from robocol_suas.campo import reglas
from robocol_suas.campo import rustyponds as campo


def test_home_y_area_de_busqueda_dentro_del_limite_de_vuelo():
    assert reglas.dentro_de(campo.LIMITE_VUELO, campo.HOME_LAT, campo.HOME_LON)
    for area in campo.AREAS_BUSQUEDA.values():
        for lat, lon in area:
            assert reglas.dentro_de(campo.LIMITE_VUELO, lat, lon)


def test_el_suelo_cubre_el_limite_de_vuelo_y_la_capa_mas_fina_el_area():
    abajo, arriba = campo.CAPAS_SUELO[0], campo.CAPAS_SUELO[-1]
    for lat, lon in campo.LIMITE_VUELO:
        e, n = campo.a_enu(lat, lon)
        assert abajo["este"][0] < e < abajo["este"][1] and abajo["norte"][0] < n < abajo["norte"][1]
    for area in campo.AREAS_BUSQUEDA.values():
        for lat, lon in area:
            e, n = campo.a_enu(lat, lon)
            assert arriba["este"][0] <= e <= arriba["este"][1] and arriba["norte"][0] <= n <= arriba["norte"][1]


def test_blancos_en_el_area_de_busqueda_y_coordenadas_coherentes():
    for nombre, (lat, lon) in campo.BLANCOS.items():
        assert any(reglas.dentro_de(area, lat, lon) for area in campo.AREAS_BUSQUEDA.values()), nombre
        e, n = campo.a_enu(lat, lon)
        assert math.isclose(e, campo.POSICIONES_BLANCOS[nombre][0], abs_tol=0.01)
        assert math.isclose(n, campo.POSICIONES_BLANCOS[nombre][1], abs_tol=0.01)


def test_escena_suelo_y_blancos_donde_dice_el_campo():
    faltan = [c["archivo"] for c in campo.CAPAS_SUELO if not os.path.exists(os.path.join(DATOS, c["archivo"]))]
    if faltan:
        pytest.skip(f"faltan fotos del suelo: {faltan}")
    modelo = modelos.cargar()
    alturas = []
    for capa in campo.CAPAS_SUELO:
        plano = modelo.geom(capa["nombre"])
        (e0, e1), (n0, n1) = capa["este"], capa["norte"]
        assert np.allclose(plano.pos[:2], [(e0 + e1) / 2, (n0 + n1) / 2], atol=0.01)
        assert np.allclose(plano.size[:2], [(e1 - e0) / 2, (n1 - n0) / 2], atol=0.01)
        alturas.append(plano.pos[2])
    assert alturas == sorted(alturas) and len(set(alturas)) == len(alturas)   # cada capa más arriba que la anterior
    for nombre, (e, n, _) in campo.POSICIONES_BLANCOS.items():
        assert np.allclose(modelo.body(nombre).pos[:2], [e, n], atol=0.01), nombre
    # El dron empieza en el home (origen del mundo), apoyado en el suelo
    assert np.allclose(modelo.body("dron").pos[:2], 0)
    datos = mujoco.MjData(modelo)
    mujoco.mj_forward(modelo, datos)
