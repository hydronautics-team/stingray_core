#!/usr/bin/env bash
set -e
# set -x   # раскомментируй для отладки — покажет, какую именно команду запускает скрипт

IMAGE_NAME="stingray_core"
CONTAINER_NAME="stingray_core"

#xhost +si:localuser:root


if [ "$(docker ps -a -q -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "Container '$CONTAINER_NAME' already exists. Removing..."
    docker rm -f $CONTAINER_NAME
fi

ROS_DOMAIN_ID=1

# получим gid группы i2c если есть
I2C_GID=""
if getent group i2c >/dev/null 2>&1; then
  I2C_GID=$(getent group i2c | cut -d: -f3)
fi

pkill -f gst-launch-1.0 || true

echo "🚀 Запуск видеострима GStreamer в фоновом режиме на хосте..."

# 2. Запускаем GStreamer на хосте в фоне (&) с перенаправлением логов в файл
gst-launch-1.0 -v v4l2src device=/dev/video0 ! \
    image/jpeg,width=640,height=480,framerate=30/1 ! \
    rtpjpegpay ! \
    udpsink host=10.42.0.2 port=5000 sync=false > /tmp/gstreamer.log 2>&1 &

echo "🐳 Запускаем Docker-контейнер..."

# запускаем контейнер от текущего пользователя, чтобы файлы в bind-mount были твоими
docker run -it --rm \
  --privileged \
  --name "$CONTAINER_NAME" \
  --network host \
  -v "$(pwd)":/stingray_core \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev\
  -e DISPLAY="$DISPLAY" \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  "$IMAGE_NAME"
