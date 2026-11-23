"""Sube el polígono del límite de vuelo del campo a ArduPilot (la geocerca).

Los parámetros de la geocerca (techo, piso, qué hacer al salirse) están en params/geocerca.parm.
El polígono no cabe en parámetros: se sube por MAVLink con el mismo protocolo que una misión
(MISSION_COUNT → el autopiloto pide cada punto → MISSION_ACK), pero marcado como "geocerca".
Es lo mismo que hace QGroundControl o Mission Planner al subir una geocerca, y sirve igual para la Pixhawk real.

    python3 -m robocol_suas.campo.geocerca [--conexion tcp:127.0.0.1:5760]

scripts/simular.sh lo ejecuta solo al arrancar SITL, por el puerto 5763 (SERIAL2), que nadie más usa.
"""
import argparse
import time

from pymavlink import mavutil

from robocol_suas.campo import rustyponds as campo

TIPO_GEOCERCA = mavutil.mavlink.MAV_MISSION_TYPE_FENCE


def conectar(direccion, baud, esperar):
    """Conecta y espera el latido (heartbeat) del autopiloto. Con `esperar`, reintenta hasta `esperar` segundos."""
    limite = time.time() + esperar
    while True:
        try:
            # source_system 250: un número cualquiera que no use otro programa (la GCS suele ser 255)
            mav = mavutil.mavlink_connection(direccion, baud=baud, source_system=250)
            if mav.wait_heartbeat(timeout=5):
                return mav
            mav.close()
        except OSError:
            pass  # SITL todavía no abrió el puerto
        if time.time() > limite:
            raise SystemExit(f"No hay autopiloto en {direccion}")
        time.sleep(2)


def subir_poligono(mav, poligono):
    """Sube un polígono de inclusión (el dron debe quedarse DENTRO) y devuelve True si el autopiloto lo acepta."""
    n = len(poligono)
    mav.mav.mission_count_send(mav.target_system, mav.target_component, n, TIPO_GEOCERCA)
    enviados = set()
    while True:
        msg = mav.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=5)
        if msg is None:
            raise TimeoutError("El autopiloto dejó de responder mientras se subía la geocerca")
        if getattr(msg, "mission_type", TIPO_GEOCERCA) != TIPO_GEOCERCA:
            continue
        if msg.get_type() == "MISSION_ACK":
            return msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED and len(enviados) == n
        # El autopiloto pide el punto número `seq`
        lat, lon = poligono[msg.seq]
        mav.mav.mission_item_int_send(
            mav.target_system, mav.target_component,
            msg.seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            0, 0,                      # current, autocontinue (no se usan en geocercas)
            n, 0, 0, 0,                # param1: cuántos vértices tiene el polígono
            int(lat * 1e7), int(lon * 1e7), 0,  # lat/lon en grados × 10^7 (enteros: no pierden precisión)
            TIPO_GEOCERCA,
        )
        enviados.add(msg.seq)


def contar_puntos(mav):
    """Pregunta cuántos puntos de geocerca tiene guardados el autopiloto (para comprobar la subida)."""
    mav.mav.mission_request_list_send(mav.target_system, mav.target_component, TIPO_GEOCERCA)
    msg = mav.recv_match(type="MISSION_COUNT", blocking=True, timeout=5)
    # Hay que cerrar la "conversación" de descarga, si no el autopiloto se queda esperando
    mav.mav.mission_ack_send(mav.target_system, mav.target_component,
                             mavutil.mavlink.MAV_MISSION_ACCEPTED, TIPO_GEOCERCA)
    return msg.count if msg else None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--conexion", default="tcp:127.0.0.1:5760")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--esperar", type=float, default=0,
                        help="segundos que se reintenta la conexión (para lanzarlo junto con SITL)")
    args = parser.parse_args()

    mav = conectar(args.conexion, args.baud, args.esperar)
    print(f"[geocerca] Subiendo el límite de vuelo ({len(campo.LIMITE_VUELO)} puntos) ...")
    if not subir_poligono(mav, campo.LIMITE_VUELO):
        raise SystemExit("[geocerca] El autopiloto rechazó la geocerca")
    print(f"[geocerca] Lista: el autopiloto tiene {contar_puntos(mav)} puntos. "
          f"Altura permitida: {campo.ALTURA_MIN_M:.0f}-{campo.ALTURA_MAX_M:.0f} m (FENCE_ALT_MIN/MAX)")
    mav.close()


if __name__ == "__main__":
    main()
