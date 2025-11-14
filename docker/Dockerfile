FROM ros:humble-ros-base

# Установим нужные пакеты + утилиты для подсветки терминала
RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions \
    python3-smbus python3-smbus2 libi2c-dev i2c-tools \
    libasio-dev netcat socat \
    ros-humble-rclcpp ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-udp-msgs \
    bash bash-completion command-not-found less \
    python3-colorama \
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