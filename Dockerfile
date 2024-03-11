FROM ros:iron-ros-base-jammy

RUN apt update && apt install -y --no-install-recommends git wget ros-iron-ament-index-cpp libboost-system-dev ros-iron-diagnostic-updater python3-pip
RUN pip3 install pyserial

WORKDIR /additional_packages_ws/src
RUN git clone https://github.com/jinmenglei/serial.git

WORKDIR /additional_packages_ws
RUN /bin/bash -c "source /opt/ros/iron/setup.bash && cd /additional_packages_ws && colcon build"

RUN echo 'source /additional_packages_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /stingray_core
CMD ["bash"]
