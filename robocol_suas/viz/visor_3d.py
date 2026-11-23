"""Visor 3D realista: el vuelo en una ventana nativa de MuJoCo, dibujada con la tarjeta gráfica del ordenador.

La simulación (física + ArduPilot) sigue en Docker. Este visor solo MIRA: se conecta al mismo WebSocket que Foxglove
(puerto 8765), lee la posición del dron (/tf) y el empuje de cada motor (/estado), y pone el dron ahí en su propia
copia del modelo. Por eso se ve igual que la cámara del dron (suelo de alta resolución, árboles, blancos, sombras),
pero con cámara libre o de persecución, a 60 cuadros por segundo, con cielo, neblina a lo lejos y hélices girando.
Los árboles son los de la foto de dron: desde arriba son reales, de lado se ven planos.

Foxglove sigue siendo la estación de tierra (mapa, gráficas, la imagen de la cámara del dron); esto es "la vista de
afuera", como la de un video del vuelo.

    scripts/visor.sh                          # en el ordenador (fuera de Docker)
    scripts/visor.sh --url ws://IP:8765       # simulación en otro ordenador

Teclas (además de las normales de MuJoCo: ratón para girar/acercar, [ y ] para cambiar de cámara, Esc = cámara libre):
    Inicio (Home)   volver a la cámara de persecución
"""
import argparse
import json
import math
import struct
import threading
import time
from collections import deque

import mujoco
import mujoco.viewer
import numpy as np
from websockets.sync.client import connect

from robocol_suas import modelos

TECLA_INICIO = 268        # GLFW_KEY_HOME
RETRASO = 0.08            # s: se dibuja un poco en el pasado para interpolar entre dos mensajes (llegan a 30 Hz)


# ---------------------------------------------------------------------------------------------------------------
# Leer los mensajes. /tf viene en protobuf (foxglove.FrameTransform); /estado, en JSON.
# Un decodificador protobuf mínimo: solo lo necesario para FrameTransform.
# ---------------------------------------------------------------------------------------------------------------
def _varint(b, i):
    v = desplazamiento = 0
    while True:
        byte = b[i]
        i += 1
        v |= (byte & 0x7F) << desplazamiento
        if byte < 0x80:
            return v, i
        desplazamiento += 7


def _campos(b):
    """Recorre un mensaje protobuf: (número de campo, valor). Valor = bytes (submensaje/texto) o float."""
    i = 0
    while i < len(b):
        clave, i = _varint(b, i)
        campo, tipo = clave >> 3, clave & 7
        if tipo == 0:                       # varint
            v, i = _varint(b, i)
        elif tipo == 1:                     # 64 bits (double)
            (v,), i = struct.unpack_from("<d", b, i), i + 8
        elif tipo == 2:                     # con largo: texto o submensaje
            n, i = _varint(b, i)
            v, i = b[i:i + n], i + n
        elif tipo == 5:                     # 32 bits
            (v,), i = struct.unpack_from("<f", b, i), i + 4
        else:
            return
        yield campo, v


def leer_frame_transform(b):
    """foxglove.FrameTransform → (tiempo, hijo, posición xyz, cuaternión wxyz). Campos de proto3: los ceros no vienen."""
    t, hijo, pos, quat = 0.0, "", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]   # quat de protobuf: x, y, z, w
    for campo, v in _campos(b):
        if campo == 1:                                  # timestamp {sec, nsec}
            s = dict(_campos(v))
            t = s.get(1, 0) + s.get(2, 0) * 1e-9
        elif campo == 3:
            hijo = v.decode()
        elif campo == 4:
            for c, x in _campos(v):
                pos[c - 1] = x
        elif campo == 5:
            quat = [0.0, 0.0, 0.0, 0.0]
            for c, x in _campos(v):
                quat[c - 1] = x
    x, y, z, w = quat
    return t, hijo, np.array(pos), np.array([w, x, y, z])


def escuchar(url, poses, estado):
    """Hilo de red: se (re)conecta al WebSocket de la simulación y guarda las poses y el empuje que llegan."""
    while True:
        try:
            with connect(url, subprotocols=["foxglove.sdk.v1", "foxglove.websocket.v1"], max_size=None) as ws:
                estado["texto"] = f"conectado a {url}, esperando al dron"
                canales = {}            # id de suscripción → (tópico, codificación)
                for msg in ws:
                    if isinstance(msg, str):
                        m = json.loads(msg)
                        if m.get("op") == "advertise":
                            nuevas = []
                            for c in m["channels"]:
                                if c["topic"] in ("/tf", "/estado") and c["topic"] not in [v[0] for v in canales.values()]:
                                    sid = len(canales) + 1
                                    canales[sid] = (c["topic"], c["encoding"])
                                    nuevas.append({"id": sid, "channelId": c["id"]})
                            if nuevas:
                                ws.send(json.dumps({"op": "subscribe", "subscriptions": nuevas}))
                    elif msg[0] == 0x01:        # datos: opcode (1) + id de suscripción (4) + tiempo (8) + mensaje
                        (sid,) = struct.unpack_from("<I", msg, 1)
                        topico, _ = canales.get(sid, ("", ""))
                        cuerpo = msg[13:]
                        if topico == "/tf":
                            t, hijo, pos, quat = leer_frame_transform(cuerpo)
                            if hijo == "dron":
                                poses.append((t, pos, quat))
                                estado["texto"] = f"recibiendo el dron de {url}"
                        elif topico == "/estado":
                            estado["empuje"] = json.loads(cuerpo).get("empuje_N", estado["empuje"])
        except OSError as error:
            estado["texto"] = f"sin conexión con {url} ({error.__class__.__name__}), reintentando…"
        time.sleep(1.0)


# ---------------------------------------------------------------------------------------------------------------
# Dibujar
# ---------------------------------------------------------------------------------------------------------------
def interpolar(poses, t):
    """Pose en el instante t (tiempo de la simulación), entre los dos mensajes que lo rodean."""
    lista = list(poses)
    if not lista:
        return None
    if t <= lista[0][0]:
        return lista[0][1:]
    for (t0, p0, q0), (t1, p1, q1) in zip(lista, lista[1:]):
        if t0 <= t <= t1:
            a = (t - t0) / max(t1 - t0, 1e-6)
            if np.dot(q0, q1) < 0:          # el mismo giro tiene dos cuaterniones: se toma el más cercano
                q1 = -q1
            q = (1 - a) * q0 + a * q1
            return (1 - a) * p0 + a * p1, q / np.linalg.norm(q)
    return lista[-1][1:]


class Helices:
    """Hace girar las hélices (son piezas fijas del modelo: se les cambia la orientación en cada cuadro).

    Una hélice de verdad gira a ~5400 rpm: en video se ve como un disco borroso, no como palas. Aquí giran a una
    velocidad que se pueda ver y se vuelven semitransparentes cuando empujan, como en un video real.
    """

    def __init__(self, modelo):
        self.modelo = modelo
        dron = modelo.body("dron").id
        motores = [modelo.site(f"m{i}").pos[:2] for i in range(1, 5)]
        sentido = [1, 1, -1, -1]            # m1 y m2 giran CCW (vistas desde arriba); m3 y m4, CW
        self.palas = []                     # (geom, ángulo inicial, sentido)
        for g in range(modelo.ngeom):
            if modelo.geom_bodyid[g] == dron and abs(modelo.geom_size[g][0] - 0.203) < 1e-3:
                motor = int(np.argmin([np.linalg.norm(modelo.geom_pos[g][:2] - m) for m in motores]))
                q = modelo.geom_quat[g]
                angulo = 2 * math.atan2(q[3], q[0])
                self.palas.append((g, angulo, sentido[motor], motor))
        self.giro = np.zeros(4)
        self.color = modelo.geom_rgba[self.palas[0][0]].copy() if self.palas else None

    def actualizar(self, empuje, dt):
        for g, angulo0, sentido, motor in self.palas:
            f = empuje[motor] if motor < len(empuje) else 0.0
            # ~6 vueltas por segundo en vuelo estacionario (11 N); 0 con el motor apagado
            self.giro[motor] += sentido * dt * 2 * math.pi * 6 * math.sqrt(max(f, 0) / 11.0)
            a = angulo0 + self.giro[motor]
            self.modelo.geom_quat[g] = [math.cos(a / 2), 0, 0, math.sin(a / 2)]
            rgba = self.color.copy()
            if rgba[3] == 0:                # la pieza usa el color de su material
                rgba = self.modelo.mat_rgba[self.modelo.geom_matid[g]].copy()
            rgba[3] = 1.0 if f < 0.5 else 0.35
            self.modelo.geom_rgba[g] = rgba


def camara_persecucion(camara, id_dron):
    camara.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camara.trackbodyid = id_dron
    camara.distance = 7.0
    camara.elevation = -18.0
    camara.azimuth = 90.0                    # mirando hacia el norte, desde el sur


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", default="ws://localhost:8765")
    args = parser.parse_args()

    modelo = modelos.cargar()   # la misma escena que la simulación
    # Ajustes solo para el visor (la cámara del dron usa los de dron.xml):
    #  - plano de corte cercano a 1 m: el visor mira de lejos, y así el suelo de alta resolución no "parpadea"
    #    contra la foto de abajo, que está solo 2 cm más baja (z-fighting)
    #  - sombras en ±200 m alrededor del dron (en vez de ±70 m): para vistas lejanas
    modelo.vis.map.znear = 1.0 / modelo.stat.extent
    modelo.vis.map.shadowclip = 200.0 / modelo.stat.extent
    datos = mujoco.MjData(modelo)
    id_dron = modelo.body("dron").id
    adr = modelo.jnt_qposadr[modelo.joint("dron").id]
    id_sol = modelo.light("sol").id
    hacia_sol = -modelo.light_dir[id_sol] / np.linalg.norm(modelo.light_dir[id_sol])
    helices = Helices(modelo)

    poses = deque(maxlen=60)
    estado = {"texto": "", "empuje": [0.0] * 4}
    threading.Thread(target=escuchar, args=(args.url, poses, estado), daemon=True).start()

    pedir_persecucion = [True]

    def tecla(codigo):
        if codigo == TECLA_INICIO:
            pedir_persecucion[0] = True

    print("Cargando el modelo (el suelo de alta resolución tarda unos segundos) ...", flush=True)
    with mujoco.viewer.launch_passive(modelo, datos, key_callback=tecla,
                                      show_left_ui=False, show_right_ui=False) as visor:
        with visor.lock():
            visor.user_scn.flags[mujoco.mjtRndFlag.mjRND_FOG] = True        # neblina a lo lejos (esconde el borde)
            visor.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = True
        desfase = None          # reloj local - tiempo de la simulación
        ultimo_texto, antes = None, time.monotonic()
        while visor.is_running():
            ahora = time.monotonic()
            dt, antes = ahora - antes, ahora
            if poses:
                t_ultimo = poses[-1][0]
                # Sincronizar el reloj local con el de la simulación (si se reinicia, se vuelve a sincronizar)
                if desfase is None or abs((ahora - desfase) - t_ultimo) > 1.0:
                    desfase = ahora - t_ultimo
                pose = interpolar(poses, ahora - desfase - RETRASO)
                if pose is not None:
                    datos.qpos[adr:adr + 3], datos.qpos[adr + 3:adr + 7] = pose
            with visor.lock():
                if pedir_persecucion[0]:
                    camara_persecucion(visor.cam, id_dron)
                    pedir_persecucion[0] = False
                helices.actualizar(estado["empuje"], dt)
                # El sol (y su cuadrado de sombras de ±70 m) se pone encima del dron, como en sim/camara.py
                modelo.light_pos[id_sol] = [datos.qpos[adr], datos.qpos[adr + 1], 0.0] + 200 * hacia_sol
                mujoco.mj_kinematics(modelo, datos)
                mujoco.mj_camlight(modelo, datos)
            visor.sync()
            if estado["texto"] != ultimo_texto:
                print(estado["texto"], flush=True)
                ultimo_texto = estado["texto"]
            time.sleep(max(0.0, 1 / 60 - (time.monotonic() - ahora)))


if __name__ == "__main__":
    main()
