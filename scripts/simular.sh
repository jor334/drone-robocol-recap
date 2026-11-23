#!/usr/bin/env bash
# Arranca la simulación completa: el puente MuJoCo (física + Foxglove) y ArduCopter SITL (piloto automático).
#
#   scripts/simular.sh                 # con cámara
#   scripts/simular.sh --sin-camara    # sin cámara (los argumentos van al puente)
#   scripts/simular.sh --viento 8,200 --turbulencia moderada   # más viento
#
# Después, en otra terminal: python3 -m robocol_suas.mision.despegue_basico
# Ctrl+C para parar las dos cosas.
set -euo pipefail
cd "$(dirname "$0")/.."

# Las fotos del suelo no están en el repo: se descargan la primera vez en datos/texturas/ (unos minutos)
python3 -c 'from robocol_suas.campo import rustyponds; rustyponds.preparar()'

# Punto de despegue: lat,lon,alt(m),rumbo (el home del campo, que también es el origen del mundo de MuJoCo)
export HOME_SUAS="${HOME_SUAS:-$(python3 -c 'from robocol_suas.campo import rustyponds; print(",".join(map(str, rustyponds.HOME)))')}"

python3 -m robocol_suas.sim.puente_sitl "$@" &
PUENTE=$!
# Sube el polígono de la geocerca en cuanto SITL arranque (por SERIAL2 = TCP 5763, que no usa nadie más)
python3 -m robocol_suas.campo.geocerca --conexion tcp:127.0.0.1:5763 --esperar 60 &
GEOCERCA=$!
trap 'kill $PUENTE $GEOCERCA 2>/dev/null' EXIT

# SITL guarda sus parámetros (eeprom.bin) y logs en la carpeta donde corre: datos/sitl/
mkdir -p datos/sitl && cd datos/sitl
# --model JSON:127.0.0.1  la física la hace un programa externo (el puente) en esta dirección, puerto 9002
# --wipe                  empieza siempre con los parámetros de los archivos .parm (sin restos de otras corridas)
# --serial0 tcp:0         MAVLink en TCP 5760 (DroneKit) sin esperar a que alguien se conecte.
#                         SERIAL1 queda en TCP 5762, para QGroundControl / Mission Planner desde fuera.
arducopter \
  --model JSON:127.0.0.1 \
  --speedup 1 \
  --home "$HOME_SUAS" \
  --defaults "$ARDUPILOT/Tools/autotest/default_params/copter.parm,../../params/robocol_quad.parm,../../params/geocerca.parm,../../params/sensores_reales.parm" \
  --wipe \
  --serial0 tcp:0 \
  -I0
