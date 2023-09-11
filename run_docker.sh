docker run --rm -it \
    -v $HOME/stingray_core:/stingray_core \
    --net=host \
    stingray:latest \
    bash
