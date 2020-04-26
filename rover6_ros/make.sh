#!/usr/bin/env bash

BASE_DIR=$(realpath "$(dirname $0)")

${BASE_DIR}/copy.sh
cd $HOME/ros_ws
catkin_make
# catkin_make  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.7m -DPYTHON_LIBRARY=/usr/lib/arm-linux-gnueabihf/libpython3.7m.so.1.0
catkin_make -DCATKIN_BLACKLIST_PACKAGES="rtabmap_ros"
cd -
