#!/bin/bash
set -e

IMAGE_NAME=stingray_core_rpi
CONTAINER_NAME=stingray_core_rpi

echo "[INFO] Запуск контейнера $CONTAINER_NAME..."

docker run -it --rm \
  --name $CONTAINER_NAME \
  --network host \
  -v $(pwd):/stingray_core \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --device /dev/i2c-0 \
  --device /dev/i2c-1 \
  $IMAGE_NAME
