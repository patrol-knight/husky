# dummy_robot only -- aggregator_node/zenoh_bridge_node run in their own
# container, built from modules/zenoh_bridge's own docker/Dockerfile.
#
# x86_64-only: PyPI's lanelet2 wheel (needed by lanelet_router.py to plan
# routes across the map) has no aarch64 build. This is a dev-machine test
# harness, never something that runs on the Orin.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir lanelet2 pyproj

WORKDIR /workspace
COPY src/dummy_robot src/dummy_robot
RUN . /opt/ros/humble/setup.sh && colcon build

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "run", "dummy_robot", "dummy_robot_node", "--ros-args", "--params-file", "/config/dummy_robot.yaml"]
