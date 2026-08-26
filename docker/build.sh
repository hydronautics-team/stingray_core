#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-stingray_core}"

echo "[INFO] Building Docker image: ${IMAGE_NAME}"

docker build \
    -t "${IMAGE_NAME}" \
    -f docker/Dockerfile \
    .

echo "[INFO] Docker image successfully built."
