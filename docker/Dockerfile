FROM ros:humble-ros-base

# Установим нужные пакеты
RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions \
    python3-smbus python3-smbus2 libi2c-dev i2c-tools \
    libasio-dev netcat socat \
    ros-humble-rclcpp ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-udp-msgs \
    && rm -rf /var/lib/apt/lists/*

# Рабочая директория
WORKDIR /stingray_core

RUN groupadd -f i2c && usermod -aG i2c root

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
