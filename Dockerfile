FROM osrf/ros:humble-desktop-full
# Add timezone
ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
# Update & Upgrade
RUN apt-get update
# Add Toolchain PPA
RUN apt-get install -y \
    software-properties-common \
    && add-apt-repository -y ppa:ubuntu-toolchain-r/test
# Install latest GCC
RUN apt-get install -y \
    gcc-13 \
    g++-13 \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-13 \
    --slave /usr/bin/gcov gcov /usr/bin/gcov-13
# Install tools
RUN apt-get update \
    && apt-get install -y \
    curl \
    wget \
    iputils-ping \
    git \
    neovim \
    unzip\
    tree \
    bash-completion
# Clear after all installs
RUN rm -rf /var/lib/apt/lists/*

# Add user
ARG NEW_USER
RUN useradd -ms /bin/bash $NEW_USER
RUN usermod -aG sudo $NEW_USER
RUN passwd -d $NEW_USER
USER $NEW_USER

RUN mkdir -p ~/.ssh

WORKDIR /stingray-core
CMD ["tail", "-f", "/dev/null"]