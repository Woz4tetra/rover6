#!/usr/bin/env bash

BASE_DIR=$(realpath "$(dirname $0)")

mkdir -p /home/pi/ros_ws/src/rover6_ros
#cp -r . ~/ros_ws/src/rover6_ros
rsync -a --delete ${BASE_DIR}/rover6_ros/* ~/ros_ws/src/rover6_ros
