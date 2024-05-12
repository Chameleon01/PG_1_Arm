#!/bin/bash

BUILD=false
DOCKERFILE=humble.Dockerfile
DOCKER=humble
XAUTH=/tmp/.docker.xauth
CONTAINER_NAME=humble_container

if [ "$BUILD" = true ]; then
     docker build -f $DOCKERFILE -t $DOCKER .
fi

xhost +si:localuser:root

docker run -it --rm \
    --device /dev/kfd \
    --device /dev/dri/renderD128 \
    --device /dev/ttyACM0 \
    --device /dev/ttyACM1 \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/francesco/Documents/PG_1_Arm/pg1_arm_ws:/home/PG_1_Arm" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    --name=$CONTAINER_NAME \
    ${DOCKER} \
    bash

echo "Done."