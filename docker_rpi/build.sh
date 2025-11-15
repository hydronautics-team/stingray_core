#!/bin/bash
set -e

IMAGE_NAME=stingray_core_rpi

echo "[INFO] Сборка Docker-образа: $IMAGE_NAME"
docker build -t --platform linux/arm32v7 $IMAGE_NAME -f docker_rpi/Dockerfile .

echo "[INFO] Образ успешно собран."
