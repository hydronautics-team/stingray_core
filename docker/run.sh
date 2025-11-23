#!/usr/bin/env bash
set -e
# set -x   # раскомментируй для отладки — покажет, какую именно команду запускает скрипт

IMAGE_NAME="stingray_core"
CONTAINER_NAME="stingray_core"

if [ "$(docker ps -a -q -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "Container '$CONTAINER_NAME' already exists. Removing..."
    docker rm -f $CONTAINER_NAME
fi

# получим gid группы i2c если есть
I2C_GID=""
if getent group i2c >/dev/null 2>&1; then
  I2C_GID=$(getent group i2c | cut -d: -f3)
fi

# запускаем контейнер от текущего пользователя, чтобы файлы в bind-mount были твоими
docker run -it --rm \
  --name "$CONTAINER_NAME" \
  --network ros2-net \
  -v "$(pwd)":/stingray_core \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY="$DISPLAY" \
  --device /dev/i2c-0 \
  --device /dev/i2c-1 \
  "$IMAGE_NAME"
