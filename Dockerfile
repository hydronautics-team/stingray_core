FROM ros:humble-ros-base-jammy

RUN apt update && apt install -y --no-install-recommends git wget nlohmann-json3-dev ros-humble-ament-index-cpp libboost-system-dev ros-humble-diagnostic-updater python3-pip
RUN pip3 install pyserial

WORKDIR /additional_packages_ws/src
RUN git clone https://github.com/jinmenglei/serial.git
# RUN git clone https://github.com/HydroScan-rov/ros2_xsens_mti_driver.git
# RUN /bin/bash -c "cd ros2_xsens_mti_driver/lib/xspublic && make -B"

WORKDIR /additional_packages_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /additional_packages_ws && colcon build"

RUN echo 'source /additional_packages_ws/install/setup.bash' >> /root/.bashrc

RUN pip3 install smbus2

WORKDIR /scanrov
CMD ["bash"]
