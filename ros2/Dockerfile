FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-numpy \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_PFE
COPY src/ src/

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
