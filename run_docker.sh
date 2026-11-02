#!/bin/bash
docker run -it --rm \
    --privileged \
    -v /dev:/dev \
    -v $(pwd):/workspace \
    --network=host \
    stingray_core
