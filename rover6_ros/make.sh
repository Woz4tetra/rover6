#!/usr/bin/env bash

BASE_DIR=$(realpath "$(dirname $0)")

${BASE_DIR}/copy.sh
cd $HOME/ros_ws
catkin_make
cd -
