FROM hydronautics/serial:humble

# clone and build core
RUN git clone -b sauvc25 https://github.com/hydronautics-team/stingray_core.git
WORKDIR /stingray_core
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /additional_packages/install/setup.bash && colcon build --packages-select stingray_core_launch stingray_core_interfaces stingray_core_communication"

RUN chmod +x  /stingray_core/docker/entrypoint.sh
ENTRYPOINT ["/stingray_core/docker/entrypoint.sh"]

RUN echo 'source /stingray_core/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
