FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-autoware-adapi-v1-msgs \
    ros-humble-autoware-planning-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY --from=patrolknight_msgs . src/patrolknight_msgs
COPY --from=state_machine . src/state_machine
RUN . /opt/ros/humble/setup.sh && colcon build --packages-up-to state_machine --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "run", "state_machine", "state_machine_node"]
