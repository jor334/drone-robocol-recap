"""Puente ArduPilot SITL ↔ MuJoCo: MuJoCo hace la física, ArduPilot es el piloto automático.

ArduPilot SITL (el firmware real de la Pixhawk compilado para el PC) tiene un modo "JSON" en el que
no simula la física él mismo: se la pide a un programa externo. Este programa. La conversación es
un ping-pong por UDP, una vez por cada paso de física que pide SITL (1200 por segundo):

    ArduPilot ──(PWM de los 4 motores)──────────────────▶ este puente (puerto 9002)
                                                              │ PWM → empuje → mj_step()
    ArduPilot ◀──(IMU, posición, actitud, velocidad en JSON)──┘

ArduPilot espera la respuesta antes de seguir (lockstep), así que la simulación y el piloto automático
nunca se desincronizan. GPS, barómetro y brújula los simula ArduPilot a partir del estado que le
mandamos (con ruido: params/sensores_reales.parm).

Además el puente agrega viento con ráfagas, arrastre del aire y error de GPS (sim/perturbaciones.py).

Uso (normalmente lo arranca scripts/simular.sh):
    python3 -m robocol_suas.sim.puente_sitl [--sin-camara] [--viento 6,200] [--turbulencia moderada] [--error-gps 1.5]
"""
import argparse
import faulthandler
import json
import os
import signal
import socket
import struct
import time

import mujoco
import numpy as np

from robocol_suas import modelos
from robocol_suas.campo import rustyponds as campo
from robocol_suas.sim import marcos
from robocol_suas.sim.perturbaciones import ErrorGPS, Viento, fuerza_aire
from robocol_suas.viz.foxglove_pub import PublicadorFoxglove


# Paquete que manda ArduPilot: magic (uint16), frecuencia (uint16), número de ciclo (uint32), 16 PWM (uint16)
PAQUETE = struct.Struct("<HHI16H")
MAGIC = 18458

# Curva de empuje de los motores: mismo exponente que MOT_THST_EXPO de ArduPilot (params/robocol_quad.parm).
# Así ArduPilot "linealiza" bien el empuje, como en el dron real bien configurado.
EXPO = 0.65


def pwm_a_empuje(pwm, empuje_max):
    """PWM (1000-2000 µs) a newtons: empuje = máx · ((1 - expo)·a + expo·a²), con a de 0 a 1."""
    a = np.clip((np.asarray(pwm, dtype=float) - 1000.0) / 1000.0, 0.0, 1.0)
    return empuje_max * ((1 - EXPO) * a + EXPO * a * a)


class Puente:
    def __init__(self, args):
        self.modelo = modelos.cargar()   # dron + campo
        self.datos = mujoco.MjData(self.modelo)
        self.empuje_max = self.modelo.actuator_ctrlrange[:, 1]
        self.home = args.home
        self.id_imu = self.modelo.site("imu").id
        self.id_dron = self.modelo.body("dron").id
        # Viento, arrastre del aire y error del GPS (sim/perturbaciones.py)
        velocidad, desde = args.viento
        self.viento = Viento(velocidad, desde, args.turbulencia)
        self.error_gps = ErrorGPS(args.error_gps)
        self.viento_ahora = np.zeros(3)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", args.puerto))
        self.socket.settimeout(1.0)

        ancho, alto = args.resolucion
        self.foxglove = PublicadorFoxglove(self.modelo, self.home, puerto=args.puerto_foxglove,
                                           ancho=ancho, alto=alto, con_camara=not args.sin_camara,
                                           efectos=not args.camara_ideal)
        self.periodo_rapido = 1.0 / 30   # transformadas, GPS, estado
        self.periodo_lento = 1.0 / args.fps_camara
        self.ultimo_ciclo = -1

    def reiniciar(self):
        mujoco.mj_resetData(self.modelo, self.datos)
        self.ultimo_ciclo = -1
        self.proximo_rapido = self.proximo_lento = 0.0
        self.foxglove.borrar_trayectoria()

    def estado_para_ardupilot(self):
        """El estado del dron en el formato y los marcos (NED/FRD) que espera ArduPilot."""
        d = self.datos
        r = marcos.rotacion_ned_frd(d.site_xmat[self.id_imu])
        estado = {
            "timestamp": d.time,
            "imu": {
                "gyro": marcos.flu_a_frd(d.sensor("giro").data).tolist(),
                "accel_body": marcos.flu_a_frd(d.sensor("acel").data).tolist(),
            },
            # posición con el error del GPS (ArduPilot la usa para simular su GPS); la verdad va a Foxglove
            "position": marcos.enu_a_ned(d.site_xpos[self.id_imu] + [*self.error_gps.error, 0.0]).tolist(),
            "attitude": list(marcos.euler_zyx(r)),
            "velocity": marcos.enu_a_ned(d.sensor("vel").data).tolist(),
        }
        # ArduPilot lee una línea JSON que empieza y termina con salto de línea
        return ("\n" + json.dumps(estado, separators=(",", ":")) + "\n").encode()

    def ciclo(self, pwm, frecuencia):
        dt = 1.0 / frecuencia
        if self.modelo.opt.timestep != dt:
            self.modelo.opt.timestep = dt
        self.datos.ctrl[:] = pwm_a_empuje(pwm[:4], self.empuje_max)
        # Viento: fuerza del aire sobre el dron, según su velocidad respecto al aire (se aplica en su centro de masa)
        d = self.datos
        velocidad = d.qvel[0:3]                       # velocidad del dron en el mundo (ENU)
        self.viento_ahora = self.viento.paso(dt, d.xpos[self.id_dron][2], float(np.linalg.norm(velocidad)))
        d.xfrc_applied[self.id_dron, :3] = fuerza_aire(velocidad, self.viento_ahora, float(d.actuator_force.sum()),
                                                       d.xmat[self.id_dron].reshape(3, 3)[:, 2])
        self.error_gps.paso(dt)
        mujoco.mj_step(self.modelo, self.datos)

    def publicar(self, pwm):
        t = self.datos.time
        if t >= self.proximo_rapido:
            self.proximo_rapido = t + self.periodo_rapido
            ned = marcos.enu_a_ned(self.datos.site_xpos[self.id_imu])
            alabeo, cabeceo, guinada = marcos.euler_zyx(marcos.rotacion_ned_frd(self.datos.site_xmat[self.id_imu]))
            self.foxglove.publicar_estado(self.datos, marcos.ned_a_geodesico(ned, *self.home[:3]), {
                "pwm": list(pwm[:4]),
                "empuje_N": self.datos.actuator_force.round(3).tolist(),
                "altura_m": round(-ned[2], 3),
                "viento_mps": self.viento_ahora.round(2).tolist(),          # (Este, Norte, Arriba)
                "error_gps_m": self.error_gps.error.round(2).tolist(),      # (Este, Norte)
                "actitud_grados": {"alabeo": np.degrees(alabeo), "cabeceo": np.degrees(cabeceo),
                                   "rumbo": np.degrees(guinada) % 360},
            })
        if t >= self.proximo_lento:
            self.proximo_lento = t + self.periodo_lento
            self.foxglove.publicar_lento(self.datos)

    def correr(self):
        self.reiniciar()
        print(f"[puente] Esperando a ArduPilot SITL en UDP {self.socket.getsockname()[1]} ...")
        conectado = False
        respuesta = None
        while True:
            try:
                paquete, origen = self.socket.recvfrom(1024)
            except socket.timeout:
                if conectado:
                    print("[puente] ArduPilot dejó de mandar paquetes; esperando ...")
                    conectado = False
                continue
            if len(paquete) != PAQUETE.size:
                continue
            magic, frecuencia, numero, *pwm = PAQUETE.unpack(paquete)
            if magic != MAGIC or frecuencia == 0:
                continue
            if not conectado:
                print(f"[puente] ArduPilot conectado desde {origen[0]}:{origen[1]} a {frecuencia} Hz")
                conectado = True

            if numero < self.ultimo_ciclo:        # ArduPilot se reinició: el dron vuelve al suelo
                print("[puente] ArduPilot se reinició; reinicio la simulación")
                self.reiniciar()
            elif numero == self.ultimo_ciclo:     # paquete repetido: se repite la respuesta, sin avanzar
                if respuesta:
                    self.socket.sendto(respuesta, origen)
                continue
            self.ultimo_ciclo = numero

            self.ciclo(pwm, frecuencia)
            respuesta = self.estado_para_ardupilot()
            self.socket.sendto(respuesta, origen)
            self.publicar(pwm)
            self.vigilar_tiempo_real()

    def vigilar_tiempo_real(self):
        """Cada 5 s compara el tiempo simulado con el real. Si el puente es lento, ArduPilot lo espera
        (lockstep) y todo va "en cámara lenta": se avisa para saberlo."""
        ahora = time.monotonic()
        if not hasattr(self, "_reloj"):
            self._reloj = (ahora, self.datos.time)
            return
        real0, sim0 = self._reloj
        if ahora - real0 >= 5.0:
            factor = (self.datos.time - sim0) / (ahora - real0)
            if factor < 0.9:
                print(f"[puente] Simulación más lenta que el tiempo real: x{factor:.2f}")
            self._reloj = (ahora, self.datos.time)


def main():
    # Diagnóstico: si el puente se queda colgado, `pkill -USR1 -f puente_sitl` imprime dónde está cada hilo
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--puerto", type=int, default=9002, help="puerto UDP del backend JSON de SITL")
    parser.add_argument("--home", type=lambda s: [float(x) for x in s.split(",")],
                        default=os.environ.get("HOME_SUAS", ",".join(map(str, campo.HOME))),
                        help="lat,lon,alt,rumbo del punto de despegue (el mismo que se le da a SITL)")
    parser.add_argument("--puerto-foxglove", type=int, default=8765)
    parser.add_argument("--sin-camara", action="store_true", help="no renderizar la cámara (más rápido)")
    parser.add_argument("--fps-camara", type=float, default=10.0)
    parser.add_argument("--viento", type=lambda s: [float(x) for x in s.split(",")], default=[4.0, 180.0],
                        help="velocidad a 10 m (m/s),de dónde viene (grados; 180 = del sur). Sin viento: 0,0")
    parser.add_argument("--turbulencia", choices=["ninguna", "ligera", "moderada", "fuerte"], default="ligera")
    parser.add_argument("--error-gps", type=float, default=1.0, help="error típico del GPS horizontal (m); 0 = perfecto")
    parser.add_argument("--camara-ideal", action="store_true",
                        help="foto sin ruido, viñeteo ni desenfoque (los defectos de una cámara real)")
    # 1280×960 a 46 m de altura (el mínimo de SUAS 2026) = 5.5 cm por píxel: un maniquí de 1.7 m ocupa ~30 píxeles.
    # Renderizar cuesta ~25 ms por imagen con OSMesa. Máximo 1920×1440 (offwidth/offheight de dron.xml).
    parser.add_argument("--resolucion", type=lambda s: tuple(int(x) for x in s.lower().split("x")),
                        default=(1280, 960), help="ancho x alto de la cámara, p. ej. 640x480 (por defecto 1280x960)")
    args = parser.parse_args()

    puente = Puente(args)
    try:
        puente.correr()
    except KeyboardInterrupt:
        pass
    finally:
        puente.foxglove.cerrar()


if __name__ == "__main__":
    main()
