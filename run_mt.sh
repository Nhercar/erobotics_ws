#!/bin/bash

# Permitir conexiones gráficas
xhost +local:docker

# Definir la imagen
IMAGE_NAME="nhcar/erobotics:v1"

# Obtener la ruta absoluta de la carpeta donde ESTÁ el script
# Esto evita que el volumen falle si lanzas el script desde otra carpeta
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

## Este escript ejecuta el contenedor para conectarse con MATLAB. Las líneas imprescindibles son:

# --ipc host ---> Es el cambio más importante. Permite que el contenedor use los segmentos de memoria 
#                 compartida del host Linux. Sin esto, el "apretón de manos" (handshake) entre el nodo de 
#                 MATLAB y el de Docker falla aunque estén en la misma red

# --volume /dev/shm:/dev/shm ------> : Fast DDS (Jazzy) usa archivos en esta ruta para el descubrimiento de nodos. 
#                                     Al mapearlo, MATLAB y Docker "leen el mismo libro de visitas".

# --env="ROS_DOMAIN_ID=0" ------> Aunque es el default, forzarlo como variable de entorno asegura que MATLAB no 
#                                 esté intentando usar otro dominio por error.

# --env="RMW_IMPLEMENTATION=rmw_fastrtps_cpp" ------> Obligamos a Jazzy a usar exactamente el mismo motor que MATLAB R2025b.

# Ejecutar el contenedor
sudo docker run -it --rm \
    --name ros2_jazzy_erobotics \
    --network host \
    --ipc host \
    --privileged \
    --device /dev/dri:/dev/dri \
    --device /dev/ttyACM*:/dev/ttyACM* \
    --device /dev/ttyUSB*:/dev/ttyUSB* \
    --group-add video \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XDG_RUNTIME_DIR=/tmp/runtime-sistemas" \
    --env="ROS_DOMAIN_ID=0" \
    --env="RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume /dev/shm:/dev/shm \
    --volume "$DIR":/home/sistemas/erobotics_ws:rw \
    --hostname sistemas \
    $IMAGE_NAME 

# 