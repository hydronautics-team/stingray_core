FROM ros:jazzy-ros-base@sha256:ed03918373b3b3a4d20d370b923bd52246eb64eafc541dba69b6a5e44925d140

RUN apt update && apt install -y --no-install-recommends git wget libboost-system-dev

WORKDIR /additional_packages/src
RUN git clone https://github.com/jinmenglei/serial.git

WORKDIR /additional_packages
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

WORKDIR /stingray_core

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x  /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc
# RUN echo 'source /additional_packages/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
