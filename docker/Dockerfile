# Entorno de Robocol SUAS: Ubuntu 24.04 + ArduPilot SITL (ArduCopter) + MuJoCo + DroneKit + Foxglove SDK.
#
# Es una receta: cada máquina construye la imagen para su propio procesador (amd64 o arm64).
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive

# Compilador (para ArduPilot), Python y Mesa/OSMesa (para que MuJoCo renderice la cámara sin pantalla)
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      git \
      python3 \
      python3-dev \
      python3-venv \
      libosmesa6 \
      libgl1 \
      libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Un entorno virtual de Python para todo (Ubuntu 24.04 no deja instalar con pip en el Python del sistema)
ENV VIRTUAL_ENV=/opt/venv
ENV PATH=/opt/venv/bin:$PATH
RUN python3 -m venv /opt/venv

# ArduPilot SITL: el firmware real de ArduCopter compilado para correr en el PC.
# Copter 4.6 era la versión estable cuando se preparó SUAS 2025.
ARG ARDUPILOT_TAG=Copter-4.6.3
# Descarga y compilación van en pasos separados: si la red falla, Docker no repite lo que ya hizo
RUN git clone --depth 1 --branch ${ARDUPILOT_TAG} --recurse-submodules --shallow-submodules \
      https://github.com/ArduPilot/ardupilot.git /opt/ardupilot
# setuptools < 80: el generador de DroneCAN de ArduPilot todavía usa pkg_resources
RUN pip install --no-cache-dir empy==3.3.4 pexpect future "setuptools<80" \
    && cd /opt/ardupilot \
    && ./waf configure --board sitl \
    && ./waf copter \
    && cp build/sitl/bin/arducopter /usr/local/bin/arducopter \
    && rm -rf build .git modules/*/.git
ENV ARDUPILOT=/opt/ardupilot

# Dependencias del proyecto
COPY docker/requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

# MuJoCo renderiza la cámara con OSMesa (por software, sin pantalla ni GPU)
ENV MUJOCO_GL=osmesa
ENV PYOPENGL_PLATFORM=osmesa
# Para poder hacer "python3 -m robocol_suas...." desde /ws
ENV PYTHONPATH=/ws

# Usuario normal (uid 1000, el habitual en Ubuntu) para que los archivos creados en el repo no sean de root
USER ubuntu
WORKDIR /ws

CMD ["sleep", "infinity"]
