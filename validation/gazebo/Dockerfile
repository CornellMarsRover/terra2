FROM osrf/ros:humble-desktop-jammy
USER root
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge \
    python3-colcon-common-extensions python3-coverage python3-pytest \
    python3-shapely python3-scipy python3-matplotlib python3-opencv \
    python3-yaml ffmpeg xvfb libxml2-utils \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /cmr/terra2
