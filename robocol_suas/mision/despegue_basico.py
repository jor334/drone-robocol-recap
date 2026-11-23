"""Misión mínima con DroneKit: armar, despegar, avanzar al norte, girar, volver al inicio y aterrizar.

Es la prueba de que todo el sistema funciona (SITL + MuJoCo + DroneKit) y la plantilla para
la misión de SUAS. DroneKit habla MAVLink con ArduPilot exactamente igual que con la Pixhawk real:
para volar el dron de verdad solo cambia la dirección de conexión.

    python3 -m robocol_suas.mision.despegue_basico [--conexion tcp:127.0.0.1:5760] [--altura 10]
"""
import argparse

from robocol_suas.mision.dronekit_compat import LocationGlobalRelative, VehicleMode, connect
from robocol_suas.mision.comandos_utilidades import distance_m, go_to, turn_to, wait_until


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--conexion", default="tcp:127.0.0.1:5760",
                        help="SITL: tcp:127.0.0.1:5760. Pixhawk por USB/telemetría: /dev/ttyACM0 (con --baud)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--altura", type=float, default=50.0, help="altura de despegue (m)")
    args = parser.parse_args()

    print(f"Conectando a {args.conexion} ...")
    dron = connect(args.conexion, baud=args.baud, wait_ready=True)
    print(f"Conectado. Firmware: {dron.version}, modo: {dron.mode.name}")

    # La EKF (el estimador de posición de ArduPilot) necesita GPS y unos segundos para converger
    print("Esperando a que el dron se pueda armar (GPS + EKF) ...")
    wait_until(lambda: dron.is_armable, "is_armable")

    dron.mode = VehicleMode("GUIDED")
    wait_until(lambda: dron.mode.name == "GUIDED", "modo GUIDED", 10)
    dron.armed = True
    wait_until(lambda: dron.armed, "armado", 10)
    print("Armado. Despegando ...")

    dron.simple_takeoff(args.altura)
    wait_until(lambda: dron.location.global_relative_frame.alt >= 0.95 * args.altura, "altura de despegue", 60)
    print(f"A {dron.location.global_relative_frame.alt:.1f} m")

    # Se guarda el punto de partida (en el aire, a la altura de despegue) para volver a él
    aqui = dron.location.global_relative_frame
    inicio = LocationGlobalRelative(aqui.lat, aqui.lon, args.altura)

    # 20 m al norte (1 grado de latitud ≈ 111 km)
    destino = LocationGlobalRelative(aqui.lat + 20 / 111_320, aqui.lon, args.altura)
    print("Avanzando 20 m al norte ...")
    go_to(dron, destino, "llegada al punto norte")
    print("Llegó al punto norte.")

    print("Girando 180° (mirando al sur) ...")
    turn_to(dron, 180)
    print(f"Rumbo: {dron.heading}°")

    print("Volviendo al punto de inicio ...")
    go_to(dron, inicio, "regreso al inicio")
    print(f"De vuelta en el inicio (a {distance_m(dron.location.global_relative_frame, inicio):.1f} m).")

    print("Aterrizando ...")
    dron.mode = VehicleMode("LAND")
    wait_until(lambda: not dron.armed, "aterrizaje y desarme", 180)
    print("En tierra y desarmado.")
    dron.close()


if __name__ == "__main__":
    main()
