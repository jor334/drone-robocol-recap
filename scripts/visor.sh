#!/usr/bin/env bash
# Visor 3D realista del vuelo (robocol_suas/viz/visor_3d.py). Se corre en el ORDENADOR, no en Docker:
# usa la tarjeta gráfica de la máquina. Necesita Python con mujoco, numpy y websockets.
#
#   scripts/visor.sh                       # simulación en este ordenador (ws://localhost:8765)
#   scripts/visor.sh --url ws://IP:8765    # simulación en otro ordenador
#
# En macOS, los visores "pasivos" de MuJoCo tienen que arrancar con mjpython (viene con el paquete mujoco).
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ "$(uname)" == "Darwin" ]]; then
  PY="${PYTHON:-python3}"
  exec "$(dirname "$("$PY" -c 'import sys; print(sys.executable)')")/mjpython" -m robocol_suas.viz.visor_3d "$@"
else
  exec python3 -m robocol_suas.viz.visor_3d "$@"
fi
