FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-smbus \
    python3-smbus2 \
    libi2c-dev \
    i2c-tools \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-udp-msgs \
    libasio-dev \
    netcat \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /stingray_core
COPY . /stingray_core
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

RUN usermod -aG i2c root

COPY /docker/entrypoint.sh /entrypoint.sh
RUN chmod +x  /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

RUN echo 'source /stingray_core/install/setup.bash' >> /root/.bashrc

CMD ["bash"]