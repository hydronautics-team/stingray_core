#!/bin/bash
set -e

IMAGE_NAME=stingray_core

echo "[INFO] Сборка Docker-образа: $IMAGE_NAME"
docker build -t $IMAGE_NAME -f docker/Dockerfile .

echo "[INFO] Образ успешно собран."
