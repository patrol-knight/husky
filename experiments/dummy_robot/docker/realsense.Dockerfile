# Plain realsense2_camera runtime -- the Husky's own launch/realsense.launch.py
# and config/realsense.yaml are bind-mounted in by docker-compose.yml, so
# this runs exactly what the Orin runs.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-humble-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "launch", "/husky/launch/realsense.launch.py"]
