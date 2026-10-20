FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-coverage \
        python3-numpy \
        python3-opencv \
        python3-pytest \
        python3-rosdep \
        ros-humble-cv-bridge \
        ros-humble-message-filters \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY src/cmr_msgs src/cmr_msgs
COPY src/astronaut_autonomy src/astronaut_autonomy
COPY scripts/test_astronaut_ros_pipeline.sh scripts/test_astronaut_ros_pipeline.sh
COPY docker/astronaut-humble/entrypoint.sh /astronaut_entrypoint.sh

RUN . /opt/ros/humble/setup.sh \
    && colcon build --symlink-install \
        --packages-select cmr_msgs astronaut_autonomy \
    && . install/setup.sh \
    && python3 -c "from cmr_msgs.msg import AstronautCommand, DriveCommand"

ENTRYPOINT ["/astronaut_entrypoint.sh"]
CMD ["bash"]
