"""Envía el estado de la simulación a Foxglove (WebSocket en el puerto 8765).

Foxglove no necesita ROS: el SDK de Foxglove abre el servidor y aquí se publica en "tópicos" con los
tipos de mensaje estándar de Foxglove. En Foxglove.app: Open connection → Foxglove WebSocket → ws://localhost:8765

Tópicos:
    /tf                     FrameTransform  mundo → dron, mundo → camara (panel 3D)
    /escena/dron            SceneUpdate     la figura del dron, sacada de las piezas del modelo de MuJoCo (panel 3D)
    /escena/trayectoria     SceneUpdate     línea con el camino recorrido (panel 3D)
    /gps                    LocationFix     posición real del dron en lat/lon (panel Map)
    /camara/imagen          CompressedImage lo que ve la cámara de abajo (panel Image)
    /camara/calibracion     CameraCalibration  parámetros de la cámara (para proyectar píxeles ↔ suelo)
    /estado                 JSON            PWM de ArduPilot, empuje de cada motor, actitud (panel Plot / Raw Messages)
    /escena/campo           SceneUpdate     límite de vuelo (rojo) y áreas de búsqueda (amarillo) de SUAS 2026 (panel 3D)
    /campo                  GeoJson         lo mismo, sobre el mapa satelital, más el maniquí y la tienda (panel Map)
    /escena/blancos         SceneUpdate     maniquí, tienda y basura de distracción del área de búsqueda (panel 3D)

Todo lo publicado es la "verdad" del simulador, no lo que estima ArduPilot.

La cámara corre en su PROPIO hilo: con sombras, una foto tarda ~80 ms (OSMesa, sin tarjeta gráfica). Si se
sacara en el lazo de la física, ArduPilot (que va en lockstep) tendría que esperarla y todo iría en cámara lenta.
Así, la física no espera: el hilo saca la foto del último estado que le pasaron y, si todavía está ocupado
con la anterior, ese cuadro se salta.
"""
import json
import math
import threading
from collections import deque

import cv2
import foxglove
import mujoco
import numpy as np
from foxglove.messages import (
    CameraCalibration, Color, CompressedImage, CubePrimitive, CylinderPrimitive, FrameTransform, GeoJson,
    LinePrimitive, LinePrimitiveLineType, LocationFix, Point3, Pose, Quaternion, SceneEntity, SceneUpdate,
    SpherePrimitive,
    Timestamp, Vector3,
)
from robocol_suas.campo import rustyponds as campo
from robocol_suas.sim.camara import EfectosDeCamara, fotografiar
from robocol_suas.vision import Vision

# MuJoCo: la cámara mira hacia su -z con y hacia arriba.
# Foxglove (convención "óptica"): mira hacia +z con y hacia abajo. Es girar 180° sobre x.
CAMARA_MUJOCO_A_OPTICA = np.diag([1.0, -1.0, -1.0])


def _tiempo(segundos):
    return Timestamp(int(segundos), int((segundos % 1) * 1e9))


def _cuaternion(mat):
    q = np.empty(4)  # MuJoCo: (w, x, y, z)
    mujoco.mju_mat2Quat(q, np.asarray(mat).flatten())
    return Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])


def _pose(x=0.0, y=0.0, z=0.0):
    return Pose(position=Vector3(x=x, y=y, z=z), orientation=Quaternion(w=1.0))


def _pose_en_cuerpo(modelo, cuerpo, raiz):
    """Posición y orientación de `cuerpo` vista desde `raiz` (sus cuerpos hijos van pegados, sin articulación).

    En MuJoCo cada cuerpo guarda su pose respecto a su padre: se van componiendo subiendo hasta la raíz.
    """
    pos, quat = np.zeros(3), np.array([1.0, 0, 0, 0])
    while cuerpo != raiz:
        nueva_pos, nuevo_quat = np.zeros(3), np.zeros(4)
        mujoco.mju_rotVecQuat(nueva_pos, pos, modelo.body_quat[cuerpo])
        nueva_pos += modelo.body_pos[cuerpo]
        mujoco.mju_mulQuat(nuevo_quat, modelo.body_quat[cuerpo], quat)
        pos, quat, cuerpo = nueva_pos, nuevo_quat, modelo.body_parentid[cuerpo]
    return pos, quat


def _figura(modelo, id_cuerpo):
    """Las piezas (geoms) de un cuerpo de MuJoCo y de todos sus hijos, convertidas en figuras de Foxglove.

    Así el 3D siempre muestra lo que hay en el modelo: si se cambia el XML, cambia solo.
    Sirve para el dron (con la caja central girada 45°) y para los blancos del área de búsqueda.
    MuJoCo da el tamaño a la mitad (semiejes, radio) y Foxglove lo quiere completo (lados, diámetro).
    """
    def es_del_dron(cuerpo):
        while cuerpo > 0:
            if cuerpo == id_cuerpo:
                return True
            cuerpo = modelo.body_parentid[cuerpo]
        return False

    cubos, cilindros, esferas = [], [], []
    for g in range(modelo.ngeom):
        cuerpo = modelo.geom_bodyid[g]
        if not es_del_dron(cuerpo):
            continue
        pos_cuerpo, quat_cuerpo = _pose_en_cuerpo(modelo, cuerpo, id_cuerpo)
        pos, quat = np.zeros(3), np.zeros(4)
        mujoco.mju_rotVecQuat(pos, modelo.geom_pos[g], quat_cuerpo)
        pos += pos_cuerpo
        mujoco.mju_mulQuat(quat, quat_cuerpo, modelo.geom_quat[g])
        w, x, y, z = quat
        pose = Pose(position=Vector3(**dict(zip("xyz", pos.tolist()))), orientation=Quaternion(x=x, y=y, z=z, w=w))
        r, gg, b, a = (modelo.mat_rgba[modelo.geom_matid[g]] if modelo.geom_matid[g] >= 0 else modelo.geom_rgba[g])
        color = Color(r=float(r), g=float(gg), b=float(b), a=float(a))
        tamano = modelo.geom_size[g]
        tipo = int(modelo.geom_type[g])  # int(): numpy.int32 no se compara bien con los enum de MuJoCo
        if tipo == mujoco.mjtGeom.mjGEOM_BOX:
            cubos.append(CubePrimitive(pose=pose, size=Vector3(x=2 * tamano[0], y=2 * tamano[1], z=2 * tamano[2]),
                                       color=color))
        elif tipo in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
            largo = 2 * tamano[1] + (2 * tamano[0] if tipo == mujoco.mjtGeom.mjGEOM_CAPSULE else 0)
            cilindros.append(CylinderPrimitive(pose=pose, size=Vector3(x=2 * tamano[0], y=2 * tamano[0], z=largo),
                                               bottom_scale=1, top_scale=1, color=color))
        elif tipo == mujoco.mjtGeom.mjGEOM_SPHERE:
            esferas.append(SpherePrimitive(pose=pose, size=Vector3(x=2 * tamano[0], y=2 * tamano[0], z=2 * tamano[0]),
                                           color=color))
        elif tipo == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
            esferas.append(SpherePrimitive(pose=pose, size=Vector3(x=2 * tamano[0], y=2 * tamano[1], z=2 * tamano[2]),
                                           color=color))
    return cubos, cilindros, esferas


def _contorno(poligono, alturas, color):
    """Un polígono GPS como líneas cerradas en el 3D (en metros respecto al home), una por cada altura."""
    puntos = [campo.a_enu(lat, lon) for lat, lon in poligono]
    puntos.append(puntos[0])
    return [LinePrimitive(type=LinePrimitiveLineType.LineStrip, pose=_pose(), thickness=2, scale_invariant=True,
                          points=[Point3(x=e, y=n, z=z) for e, n in puntos], color=color)
            for z in alturas]


def _escena_campo():
    """Límite de vuelo en rojo (en el suelo, en el piso y en el techo de altura) y áreas de búsqueda en amarillo.

    Se dibuja respecto al home de campo.py: si se arranca con otro HOME_SUAS, no coincide con el suelo.
    """
    rojo = Color(r=1.0, g=0.2, b=0.2, a=0.9)
    amarillo = Color(r=1.0, g=0.85, b=0.1, a=1.0)
    lineas = _contorno(campo.LIMITE_VUELO, [0.3, campo.ALTURA_MIN_M, campo.ALTURA_MAX_M], rojo)
    for poligono in campo.AREAS_BUSQUEDA.values():
        lineas += _contorno(poligono, [0.3], amarillo)
    return lineas


def _geojson_campo():
    """Lo mismo para el panel Map. GeoJSON escribe las coordenadas al revés: [lon, lat]."""
    def poligono(puntos, nombre, color):
        anillo = [[lon, lat] for lat, lon in puntos] + [[puntos[0][1], puntos[0][0]]]
        return {"type": "Feature", "properties": {"name": nombre, "style": {"color": color, "fillOpacity": 0.05}},
                "geometry": {"type": "Polygon", "coordinates": [anillo]}}
    elementos = [poligono(campo.LIMITE_VUELO, "Límite de vuelo", "#ff3333")]
    elementos += [poligono(p, f"Área de búsqueda {n}", "#ffd919") for n, p in campo.AREAS_BUSQUEDA.items()]
    # La posición real de los blancos (la "respuesta correcta" del simulador, para comparar con lo que se detecte)
    elementos += [{"type": "Feature", "properties": {"name": nombre, "style": {"color": "#ff66ff"}},
                   "geometry": {"type": "Point", "coordinates": [lon, lat]}}
                  for nombre, (lat, lon) in campo.BLANCOS.items()]
    return json.dumps({"type": "FeatureCollection", "features": elementos})


class PublicadorFoxglove:
    def __init__(self, modelo, home, puerto=8765, ancho=1280, alto=960, con_camara=True, efectos=True):
        self.modelo = modelo
        self.home = home  # (lat, lon, alt)
        self.servidor = foxglove.start_server(name="robocol_suas", host="0.0.0.0", port=puerto)
        self.id_dron = modelo.body("dron").id
        self.id_camara = modelo.camera("camara_abajo").id

        self.ancho, self.alto = ancho, alto
        self.vision = Vision()
        # Camino recorrido: un punto cada vez que el dron se mueve más de 20 cm (los últimos 5000)
        self.trayectoria = deque(maxlen=5000)
        # El campo no cambia: se calcula una vez y se reenvía cada segundo (para quien se conecte tarde)
        self.lineas_campo = _escena_campo()
        self.geojson_campo = _geojson_campo()
        self.proximo_campo = 0.0
        self.figura_dron = _figura(modelo, self.id_dron)
        self.figura_blancos = _figura(modelo, modelo.body("blancos").id)

        # Hilo de la cámara: tiene su propia copia del estado (datos_camara) y su propio renderizador
        self.con_camara = con_camara
        self.efectos = EfectosDeCamara(ancho, alto) if efectos else None
        self.datos_camara = mujoco.MjData(modelo)
        self.foto_pedida = threading.Event()
        self.cerrando = False
        self.ocupada = False
        if con_camara:
            self.hilo_camara = threading.Thread(target=self._hilo_camara, name="camara", daemon=True)
            self.hilo_camara.start()

    def borrar_trayectoria(self):
        self.trayectoria.clear()

    def cerrar(self):
        self.cerrando = True
        self.foto_pedida.set()
        self.servidor.stop()

    def publicar_estado(self, datos, lat_lon_alt, extra):
        """Lo rápido: transformadas, GPS y estado (llamar a ~30 Hz)."""
        t = _tiempo(datos.time)
        foxglove.log("/tf", FrameTransform(
            timestamp=t, parent_frame_id="mundo", child_frame_id="dron",
            translation=Vector3(**dict(zip("xyz", datos.xpos[self.id_dron]))),
            rotation=_cuaternion(datos.xmat[self.id_dron]),
        ))
        foxglove.log("/tf", FrameTransform(
            timestamp=t, parent_frame_id="mundo", child_frame_id="camara",
            translation=Vector3(**dict(zip("xyz", datos.cam_xpos[self.id_camara]))),
            rotation=_cuaternion(datos.cam_xmat[self.id_camara].reshape(3, 3) @ CAMARA_MUJOCO_A_OPTICA),
        ))
        lat, lon, alt = lat_lon_alt
        foxglove.log("/gps", LocationFix(timestamp=t, frame_id="dron", latitude=lat, longitude=lon, altitude=alt))
        foxglove.log("/estado", extra)

    def publicar_lento(self, datos):
        """Lo pesado: la figura del dron (para quien se conecte tarde) y la cámara (llamar a ~5-10 Hz)."""
        t = _tiempo(datos.time)
        if datos.time >= self.proximo_campo:
            self.proximo_campo = datos.time + 1.0
            foxglove.log("/escena/campo", SceneUpdate(entities=[SceneEntity(
                timestamp=t, frame_id="mundo", id="campo", lines=self.lineas_campo)]))
            foxglove.log("/campo", GeoJson(geojson=self.geojson_campo))
            # "blancos" está en el origen del mundo: sus piezas ya vienen en coordenadas del mundo
            foxglove.log("/escena/blancos", SceneUpdate(entities=[SceneEntity(
                timestamp=t, frame_id="mundo", id="blancos", cubes=self.figura_blancos[0],
                cylinders=self.figura_blancos[1], spheres=self.figura_blancos[2])]))
        foxglove.log("/escena/dron", SceneUpdate(entities=[SceneEntity(
            timestamp=t, frame_id="dron", id="dron", frame_locked=True,
            cubes=self.figura_dron[0], cylinders=self.figura_dron[1], spheres=self.figura_dron[2],
        )]))

        posicion = datos.xpos[self.id_dron].copy()
        if not self.trayectoria or np.linalg.norm(posicion - self.trayectoria[-1]) > 0.2:
            self.trayectoria.append(posicion)
        if len(self.trayectoria) >= 2:
            foxglove.log("/escena/trayectoria", SceneUpdate(entities=[SceneEntity(
                timestamp=t, frame_id="mundo", id="trayectoria",
                lines=[LinePrimitive(
                    type=LinePrimitiveLineType.LineStrip, pose=_pose(), thickness=3, scale_invariant=True,
                    points=[Point3(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in self.trayectoria],
                    color=Color(r=1.0, g=0.75, b=0.0, a=1.0),
                )],
            )]))

        # Pedirle una foto al hilo de la cámara (si está libre) con una copia del estado actual
        if self.con_camara and not self.ocupada:
            self.datos_camara.time = datos.time
            self.datos_camara.qpos[:] = datos.qpos
            self.ocupada = True
            self.foto_pedida.set()

    def _hilo_camara(self):
        # El renderizador se crea aquí: un contexto de OpenGL solo se puede usar desde el hilo que lo creó
        try:
            renderizador = mujoco.Renderer(self.modelo, self.alto, self.ancho)
        except Exception as error:  # sin OpenGL (p. ej. falta OSMesa): la simulación sigue sin cámara
            print(f"[foxglove] Sin cámara: no se pudo renderizar ({error})")
            self.con_camara = False
            return
        # Cámara ideal (sin distorsión): f = (alto/2) / tan(fovy/2)
        f = (self.alto / 2) / math.tan(math.radians(self.modelo.cam_fovy[self.id_camara]) / 2)
        cx, cy = self.ancho / 2, self.alto / 2
        while True:
            self.foto_pedida.wait()
            self.foto_pedida.clear()
            if self.cerrando:
                break
            datos = self.datos_camara
            mujoco.mj_kinematics(self.modelo, datos)   # posiciones de cuerpos y cámara a partir de qpos
            mujoco.mj_camlight(self.modelo, datos)
            t = _tiempo(datos.time)
            rgb = fotografiar(renderizador, datos, self.id_camara, self.efectos)
            self.ocupada = False                       # el estado ya se usó: se puede pedir la siguiente
            self.vision.process_frame(rgb)
            ok, jpg = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                foxglove.log("/camara/imagen", CompressedImage(timestamp=t, frame_id="camara", data=jpg.tobytes(),
                                                               format="jpeg"))
            foxglove.log("/camara/calibracion", CameraCalibration(
                timestamp=t, frame_id="camara", width=self.ancho, height=self.alto, distortion_model="plumb_bob",
                D=[0.0] * 5, K=[f, 0, cx, 0, f, cy, 0, 0, 1], R=[1, 0, 0, 0, 1, 0, 0, 0, 1],
                P=[f, 0, cx, 0, 0, f, cy, 0, 0, 0, 1, 0],
            ))
        renderizador.close()
