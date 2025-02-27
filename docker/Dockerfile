FROM hydronautics/serial:humble

WORKDIR /stingray_core
COPY . /stingray_core
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /additional_packages/install/setup.bash && colcon build --packages-select stingray_core_launch stingray_core_interfaces stingray_core_communication"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x  /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

RUN echo 'source /stingray_core/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
