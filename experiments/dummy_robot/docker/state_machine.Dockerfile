# The teammate's state_machine (modules/state_machine), unchanged: /robot_status and /robot_mode.
FROM ros:humble-ros-base

WORKDIR /workspace
COPY --from=state_machine . src/state_machine
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select state_machine

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "run", "state_machine", "state_machine_node"]
