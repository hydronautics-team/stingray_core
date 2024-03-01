docker run --rm -it \
    -v $HOME/stingray_core:/stingray_core \
    --device=/dev/ttyS0 \
    --device=/dev/ttyS1 \
    stingray_core:latest \
    bash
