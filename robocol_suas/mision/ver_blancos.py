"""Vuelo de inspección: pasar por encima de los blancos de SUAS 2026 para verlos con la cámara (en Foxglove).

Despega a 50 m, va hasta la tienda y luego hasta el maniquí (área de búsqueda 2), se queda quieto encima de cada uno
unos segundos, vuelve a la pista y aterriza.

OJO: usa las posiciones REALES de los blancos (campo.BLANCOS), que en la competencia nadie conoce. Es una herramienta
para revisar la simulación (¿se ven? ¿de qué tamaño?), no una misión de SUAS: esa tiene que encontrarlos con la cámara.

¿Por qué 50 m? El reglamento pide al menos 150 ft (45.7 m) y la geocerca de ArduPilot tiene el piso en 47 m.

    python3 -m robocol_suas.mision.ver_blancos [--altura 50] [--espera 15]
"""
import argparse
import time

from robocol_suas.campo import rustyponds as campo
from robocol_suas.mision.dronekit_compat import LocationGlobalRelative, VehicleMode, connect
from robocol_suas.mision.comandos_utilidades import go_to, wait_until


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--conexion", default="tcp:127.0.0.1:5760")
    parser.add_argument("--altura", type=float, default=50.0, help="altura de vuelo (m); entre 49 y 120")
    parser.add_argument("--espera", type=float, default=15.0, help="segundos quieto encima de cada blanco")
    parser.add_argument("--velocidad", type=float, default=8.0, help="m/s")
    args = parser.parse_args()

    dron = connect(args.conexion, wait_ready=True)
    print("Esperando GPS + EKF ...")
    wait_until(lambda: dron.is_armable, "is_armable", 180)
    dron.mode = VehicleMode("GUIDED")
    wait_until(lambda: dron.mode.name == "GUIDED", "modo GUIDED", 10)
    dron.armed = True
    wait_until(lambda: dron.armed, "armado", 20)
    print(f"Despegando a {args.altura:.0f} m ...")
    dron.simple_takeoff(args.altura)
    wait_until(lambda: dron.location.global_relative_frame.alt >= args.altura - 1, "altura", 120)
    aqui = dron.location.global_relative_frame
    pista = LocationGlobalRelative(aqui.lat, aqui.lon, args.altura)

    for nombre in ("tienda", "maniqui"):
        lat, lon = campo.BLANCOS[nombre]
        este, norte = campo.a_enu(lat, lon)
        print(f"Volando sobre {nombre} ({este:.0f} m al este, {norte:.0f} m al norte de la pista) ...")
        go_to(dron, LocationGlobalRelative(lat, lon, args.altura), nombre, speed=args.velocidad,
              tolerance=1.0, timeout=120)
        print(f"  Encima de {nombre}: {args.espera:.0f} s para mirar la cámara en Foxglove")
        time.sleep(args.espera)

    print("Volviendo a la pista ...")
    go_to(dron, pista, "pista", speed=args.velocidad, tolerance=1.5, timeout=120)
    print("Aterrizando ...")
    dron.mode = VehicleMode("LAND")
    wait_until(lambda: not dron.armed, "aterrizaje", 240)
    print("En tierra.")
    dron.close()


if __name__ == "__main__":
    main()
