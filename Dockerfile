FROM osrf/ros:humble-desktop-full

ARG BOOST_VERSION
ARG NEW_USER

# Add timezone
ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
# Install tools
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    iputils-ping \
    git \
    neovim \
    unzip\
    tree \
    bash-completion && \
    rm -rf /var/lib/apt/lists/*
# Add Toolchain PPA
RUN apt-get update && apt-get install -y \
    software-properties-common && \
    add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
    rm -rf /var/lib/apt/lists/*
# Install latest GCC
RUN apt-get update && apt-get install -y \
    gcc-13 \
    g++-13 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-13 \
    --slave /usr/bin/gcov gcov /usr/bin/gcov-13 && \
    rm -rf /var/lib/apt/lists/*
# Install Boost. From https://leimao.github.io/blog/Boost-Docker/
RUN cd /tmp && \
    BOOST_VERSION_MOD=$(echo $BOOST_VERSION | tr . _) && \
    wget https://archives.boost.io/release/${BOOST_VERSION}/source/boost_${BOOST_VERSION_MOD}.tar.bz2 && \
    tar --bzip2 -xf boost_${BOOST_VERSION_MOD}.tar.bz2 && \
    cd boost_${BOOST_VERSION_MOD} && \
    ./bootstrap.sh --prefix=/usr/local && \
    ./b2 install && \
    rm -rf /tmp/*
# Install latest CLANG
RUN bash -c "$(curl -fsSL https://apt.llvm.org/llvm.sh)"
RUN apt-get update && apt-get install -y \
    clang-tidy-20 \
    clang-format-20 && \
    update-alternatives --install /usr/bin/clang clang /usr/bin/clang-20 100 \
    --slave /usr/bin/clangd clangd /usr/bin/clangd-20 \
    --slave /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-20 \
    --slave /usr/bin/clang-format clang-format /usr/bin/clang-format-20 && \
    rm -rf /var/lib/apt/lists/*
# Install Ruff
RUN apt-get update && apt-get install -y \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*
RUN pip install ruff
# Install python modules
RUN pip install click ipykernel ipython
# Add user
RUN useradd -ms /bin/bash $NEW_USER
RUN usermod -aG sudo $NEW_USER
RUN passwd -d $NEW_USER
USER $NEW_USER
# Prepare bash
RUN echo "# Color bash invitation" >> ~/.bashrc && \
    echo "C_GREEN='\[\033[1;32m\]'" >> ~/.bashrc && \
    echo "C_CYAN='\[\033[1;36m\]'" >> ~/.bashrc && \
    echo "C_RED='\[\033[1;31m\]'" >> ~/.bashrc && \
    echo "C_NC='\[\033[0m\]'" >> ~/.bashrc && \
    echo "export PS1=\"\${C_CYAN}\u@\${C_RED}[DOCKER]\${C_GREEN}:\w\${C_NC}\\\$ \"" >> ~/.bashrc
# Prepare ssh
RUN mkdir -p ~/.ssh

WORKDIR /stingray-core
CMD ["tail", "-f", "/dev/null"]