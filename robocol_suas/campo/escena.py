"""Arma la escena de MuJoCo: el dron (modelos/dron.xml) en el campo (campo/rustyponds.py) → datos/escena.xml.

Lo que se escribe a partir de los datos del campo:
  - el sol (dirección y sombras),
  - las capas de suelo: planos con fotos aéreas, de la más grande a la más detallada, cada una 2 cm más arriba que
    la anterior (si estuvieran a la misma altura, la cámara no sabría cuál dibujar: "z-fighting"),
  - los blancos: las piezas de modelos/blancos.xml, puestas donde dice POSICIONES_BLANCOS.

datos/escena.xml no se edita a mano: se vuelve a escribir cada vez que se carga el modelo (robocol_suas.modelos.cargar()).

    python3 -m robocol_suas.campo.escena      # solo escribirla
"""
import math
import os
import xml.etree.ElementTree as ET

from robocol_suas import DATOS, modelos
from robocol_suas.campo import rustyponds

DESTINO = os.path.join(DATOS, "escena.xml")
SEPARACION_CAPAS = 0.02   # m


def _sol(campo):
    az, el = math.radians(campo.SOL_AZIMUT_GRADOS), math.radians(campo.SOL_ELEVACION_GRADOS)
    hacia_sol = (math.sin(az) * math.cos(el), math.cos(az) * math.cos(el), math.sin(el))
    # Sobre el suelo, el sol aporta diffuse × sen(elevación): se ajusta para que sea 0.55
    # (con ambiente + linterna = 0.5, el suelo al sol recibe ~1 y a la sombra la mitad)
    difusa = 0.55 / math.sin(el)
    d = " ".join(f"{-x:.4f}" for x in hacia_sol)
    return (f'<light name="sol" pos="0 0 100" dir="{d}" directional="true" castshadow="true" '
            f'diffuse="{difusa:.3f} {difusa:.3f} {difusa:.3f}" specular="0.1 0.1 0.1"/>')


def generar(campo=rustyponds):
    """Escribe datos/escena.xml y devuelve su ruta."""
    activos, cuerpo = [], []
    # ---- Suelo ----
    for i, capa in enumerate(campo.CAPAS_SUELO):
        (e0, e1), (n0, n1) = capa["este"], capa["norte"]
        nombre = capa["nombre"]
        activos.append(f'<texture name="{nombre}" type="2d" file="{capa["archivo"]}"/>')
        # texuniform="false" + texrepeat="1 1": UNA sola copia de la imagen cubre todo el plano
        activos.append(f'<material name="{nombre}" texture="{nombre}" texuniform="false" texrepeat="1 1" '
                       f'specular="0" shininess="0"/>')
        choca = "" if i == 0 else ' contype="0" conaffinity="0"'   # solo el de abajo choca (el dron se apoya en él)
        cuerpo.append(f'<geom name="{nombre}" type="plane" pos="{(e0 + e1) / 2:.3f} {(n0 + n1) / 2:.3f} '
                      f'{i * SEPARACION_CAPAS:.2f}" size="{(e1 - e0) / 2:.3f} {(n1 - n0) / 2:.3f} 1" '
                      f'material="{nombre}"{choca}/>')
    # Más allá de las fotos, campo de color parejo hasta el horizonte (bajo la neblina; solo se ve desde el visor 3D).
    # Va 2 m más abajo para que de lejos no se mezcle con la foto.
    cuerpo.append('<geom name="suelo_lejano" type="plane" pos="0 0 -2" size="20000 20000 1" '
                  'rgba="0.30 0.36 0.24 1" contype="0" conaffinity="0"/>')
    # ---- Blancos: las piezas de la plantilla, en las posiciones del campo ----
    plantilla = ET.parse(modelos.BLANCOS).getroot()
    for hijo in plantilla.find("asset"):
        activos.append(ET.tostring(hijo, encoding="unicode").strip())
    blancos = plantilla.find("worldbody/body[@name='blancos']")
    for objeto in blancos.findall("body"):
        nombre = objeto.get("name")
        if nombre not in campo.POSICIONES_BLANCOS:
            raise ValueError(f"El campo no dice dónde va '{nombre}' (POSICIONES_BLANCOS)")
        este, norte, giro = campo.POSICIONES_BLANCOS[nombre]
        objeto.set("pos", f"{este:.3f} {norte:.3f} {(len(campo.CAPAS_SUELO) - 1) * SEPARACION_CAPAS:.2f}")
        objeto.set("euler", f"0 0 {giro}")
    cuerpo.append(ET.tostring(blancos, encoding="unicode").strip())

    dron = os.path.relpath(modelos.DRON, DATOS)
    xml = (f'<!-- GENERADO por robocol_suas/campo/escena.py. No editar a mano.\n     {campo.CREDITOS} -->\n'
           f'<mujoco model="robocol_{campo.NOMBRE}">\n  <include file="{dron}"/>\n  <asset>\n    '
           + "\n    ".join(activos) + "\n  </asset>\n  <worldbody>\n    " + _sol(campo) + "\n    "
           + "\n    ".join(cuerpo) + "\n  </worldbody>\n</mujoco>\n")
    # Se escribe en un archivo temporal y se renombra: si dos programas cargan el modelo a la vez, ninguno lee
    # un archivo a medio escribir
    os.makedirs(DATOS, exist_ok=True)
    temporal = f"{DESTINO}.{os.getpid()}"
    with open(temporal, "w") as f:
        f.write(xml)
    os.replace(temporal, DESTINO)
    return DESTINO


if __name__ == "__main__":
    print(generar())
