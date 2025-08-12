FROM hydronautics/serial:humble

RUN apt-get update && apt-get install -y \
    python3-smbus \
    python3-smbus2 \
    libi2c-dev \
    i2c-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /stingray_core
COPY . /stingray_core
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /additional_packages/install/setup.bash && colcon build --packages-select stingray_core_launch stingray_core_interfaces stingray_core_communication stingray_core_sensors"

RUN usermod -aG i2c root

COPY /docker/entrypoint.sh /entrypoint.sh
RUN chmod +x  /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

RUN echo 'source /stingray_core/install/setup.bash' >> /root/.bashrc

CMD ["bash"]