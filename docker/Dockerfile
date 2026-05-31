FROM ros:humble-ros-base

# Установим нужные пакеты + инструменты разработки для VS Code Dev Containers
RUN apt-get update && apt-get install -y \
    bash bash-completion build-essential cmake command-not-found gdb git jq less lsb-release \
    clang clangd clang-format clang-tidy \
    i2c-tools libasio-dev libboost-all-dev libi2c-dev libserial-dev netcat-openbsd socat \
    python3-colcon-common-extensions python3-colorama python3-pip python3-pyqtgraph \
    python3-rosdep python3-smbus python3-smbus2 python3-vcstool \
    ros-humble-ament-clang-tidy \
    ros-humble-ament-cmake-gtest \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-geometry-msgs \
    ros-humble-lifecycle-msgs \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-python-qt-binding \
    ros-humble-rclcpp \
    ros-humble-rclcpp-action \
    ros-humble-rclcpp-components \
    ros-humble-rclcpp-lifecycle \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-gui-py \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-tf2-geometry-msgs \
    ros-humble-udp-msgs \
    && rm -rf /var/lib/apt/lists/*

# Настройка цветного вывода и подсветки
RUN echo 'export TERM=xterm-256color' >> ~/.bashrc && \
    echo 'force_color_prompt=yes' >> ~/.bashrc && \
    echo 'export LS_OPTIONS="--color=auto"' >> ~/.bashrc && \
    echo 'eval "`dircolors`"' >> ~/.bashrc && \
    echo 'alias ls="ls $LS_OPTIONS"' >> ~/.bashrc && \
    echo 'alias ll="ls $LS_OPTIONS -l"' >> ~/.bashrc && \
    echo 'alias l="ls $LS_OPTIONS -lA"' >> ~/.bashrc && \
    echo 'alias grep="grep --color=auto"' >> ~/.bashrc && \
    echo 'alias ip="ip --color=auto"' >> ~/.bashrc && \
    echo 'export GCC_COLORS="error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01"' >> ~/.bashrc

# Цветной prompt с информацией о контейнере
RUN echo 'export PS1="\[\033[1;32m\]\u@stingray-container:\w\$\[\033[0m\] "' >> ~/.bashrc

# Настройка для подсветки синтаксиса в редакторах (опционально)
RUN echo 'set -o vi' >> ~/.bashrc && \
    echo 'set editing-mode vi' >> ~/.inputrc && \
    echo 'set show-mode-in-prompt on' >> ~/.inputrc

# ROS/workspace setup for interactive VS Code terminals.
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo '[ -f /stingray_core/install/setup.bash ] && source /stingray_core/install/setup.bash' >> ~/.bashrc

# Установка переменных окружения для цветного вывода
ENV TERM=xterm-256color
ENV PYTHONUNBUFFERED=1
ENV FORCE_COLOR=1
ENV COLCON_DEFAULT_OPTS="--event-handlers console_cohesion+"

# Рабочая директория
WORKDIR /stingray_core

RUN groupadd -f i2c && usermod -aG i2c root

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
