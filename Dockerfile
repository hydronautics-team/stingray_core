FROM ros:iron-ros-base-jammy

RUN apt update && apt install -y --no-install-recommends git wget libboost-system-dev

WORKDIR /additional_packages/src
RUN git clone https://github.com/jinmenglei/serial.git

WORKDIR /additional_packages
RUN /bin/bash -c "source /opt/ros/iron/setup.bash && colcon build"

RUN echo 'source /additional_packages/install/setup.bash' >> /root/.bashrc

WORKDIR /stingray_core
CMD ["bash"]
