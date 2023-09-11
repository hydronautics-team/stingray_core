docker run --rm -it \
    -v $HOME/stingray:/stingray \
    --net=host \
    stingray:latest \
    bash
