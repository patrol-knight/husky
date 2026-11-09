# Builds natively on both amd64 and arm64 (Orin): lanelet2 comes from the ROS apt repo.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    libyaml-cpp-dev \
    ros-humble-lanelet2-core \
    ros-humble-lanelet2-io \
    ros-humble-lanelet2-projection \
    ros-humble-lanelet2-routing \
    ros-humble-lanelet2-traffic-rules \
    ros-humble-autoware-adapi-v1-msgs \
    ros-humble-autoware-planning-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY src/dummy_robot src/dummy_robot
RUN . /opt/ros/humble/setup.sh && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "run", "dummy_robot", "dummy_robot_node", "--ros-args", "--params-file", "/config/dummy_robot.yaml"]
