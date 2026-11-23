"""Los modelos de MuJoCo escritos a mano: el dron (dron.xml) y la plantilla de los blancos (blancos.xml).

El modelo completo (dron + campo) es datos/escena.xml, que arma campo/escena.py. Para usarlo:

    from robocol_suas import modelos
    modelo = modelos.cargar()
"""
import os

CARPETA = os.path.dirname(os.path.abspath(__file__))
DRON = os.path.join(CARPETA, "dron.xml")
BLANCOS = os.path.join(CARPETA, "blancos.xml")


def ruta():
    """Escribe la escena (dron + campo) y devuelve su ruta."""
    from robocol_suas.campo import escena
    return escena.generar()


def cargar():
    """El modelo completo, listo para simular."""
    import mujoco
    return mujoco.MjModel.from_xml_path(ruta())
