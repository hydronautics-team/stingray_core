FROM mcr.microsoft.com/devcontainers/base:jammy

ARG USERNAME=hydrolib
ARG UID=1000
ARG GID=$UID

ARG CMAKE_VERSION=3.25.2
ARG CMAKE_ARCH=linux-x86_64

# Install some dependencies packages
RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add - \
    && echo "deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-19 main" \
    | sudo tee /etc/apt/sources.list.d/llvm.list \
    && apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    build-essential \
    gdb \
    libgtest-dev \
    doxygen \
    clang-format-19 \
    clangd-19 \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* \
    && update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-19 100 \
    && update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-19 100 \
    && wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-${CMAKE_ARCH}.tar.gz \
    && tar -xzf cmake-${CMAKE_VERSION}-${CMAKE_ARCH}.tar.gz \
    && cp -r cmake-${CMAKE_VERSION}-${CMAKE_ARCH}/bin /usr/local/ \
    && cp -r cmake-${CMAKE_VERSION}-${CMAKE_ARCH}/share /usr/local/ \
    && rm -rf /tmp/cmake-*
