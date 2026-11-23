"""Pruebas de las conversiones ENU/FLU (MuJoCo) <-> NED/FRD (ArduPilot).

    python3 -m pytest tests
"""
import math

import mujoco
import numpy as np

from robocol_suas import modelos
from robocol_suas.sim import marcos


def rot_z(grados):
    a = math.radians(grados)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def test_dron_mirando_al_norte_tiene_rumbo_cero():
    # En MuJoCo, "adelante" = Norte es girar 90° sobre z (ENU)
    r = marcos.rotacion_ned_frd(rot_z(90))
    np.testing.assert_allclose(r, np.eye(3), atol=1e-12)


def test_dron_mirando_al_este_tiene_rumbo_90():
    _, _, guinada = marcos.euler_zyx(marcos.rotacion_ned_frd(rot_z(0)))
    assert math.isclose(math.degrees(guinada), 90.0)


def test_nariz_abajo_es_cabeceo_negativo():
    # Mirando al norte e inclinado nariz abajo: en FLU es girar +10° sobre y (izquierda)
    a = math.radians(10)
    r_y = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    alabeo, cabeceo, guinada = marcos.euler_zyx(marcos.rotacion_ned_frd(rot_z(90) @ r_y))
    assert math.isclose(math.degrees(cabeceo), -10.0, abs_tol=1e-9)
    assert abs(alabeo) < 1e-9 and abs(guinada) < 1e-9


def test_subir_es_z_negativo_en_ned():
    np.testing.assert_allclose(marcos.enu_a_ned([1.0, 2.0, 3.0]), [2.0, 1.0, -3.0])


def test_dron_quieto_en_el_suelo_mide_menos_g_en_z():
    # El modelo completo, apoyado en el suelo sin motores: el acelerómetro (FRD) debe dar (0, 0, -9.81)
    modelo = modelos.cargar()
    datos = mujoco.MjData(modelo)
    for _ in range(400):
        mujoco.mj_step(modelo, datos)
    acel = marcos.flu_a_frd(datos.sensor("acel").data)
    np.testing.assert_allclose(acel, [0.0, 0.0, -9.81], atol=0.05)


def test_un_metro_al_norte():
    lat, lon, alt = marcos.ned_a_geodesico([1.0, 0.0, -2.0], 38.0, -76.0, 10.0)
    assert math.isclose((lat - 38.0) * 111_319.5, 1.0, rel_tol=1e-3)
    assert lon == -76.0 and alt == 12.0


def test_geodesico_a_enu_es_la_inversa():
    lat0, lon0 = 36.21433, -96.00704
    lat, lon, _ = marcos.ned_a_geodesico([350.0, -120.0, 0.0], lat0, lon0, 210.0)
    este, norte = marcos.geodesico_a_enu(lat, lon, lat0, lon0)
    assert math.isclose(este, -120.0, abs_tol=1e-6) and math.isclose(norte, 350.0, abs_tol=1e-6)
