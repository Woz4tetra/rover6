#!/usr/bin/env bash

echo "Running rover6_ros install"

BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(realpath "${BASE_DIR}/..")

ROS_INSTALL_WS=/home/pi/melodic_ws/
ROS_WS=/home/pi/ros_ws/

mkdir -p ${ROS_INSTALL_WS}
mkdir -p ${ROS_WS}
cd ${ROS_INSTALL_WS}

echo "Installing ros dependencies"
sudo apt install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential

echo "Installing package dependencies"
pip install python-empy

echo "running 'sudo rosdep init'"
sudo rosdep init

echo "running 'rosdep update'"
rosdep update

echo "Generating list of packages"
rosinstall_generator robot --rosdistro melodic --deps --tar > melodic-robot.rosinstall

echo "Initializing workspace. If this fails, run 'wstool update -j 4 -t src'"
wstool init -j4 src melodic-robot.rosinstall

# rosdep install --from-paths src --ignore-src --rosdistro melodic -y  # skipping since catkin decides to uninstall itself...

./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -j3

cd ${ROS_WS}/src
bash ${BASE_DIR}/clone_repos.sh

chmod +x ${BASE_DIR}/set_bashrc.sh
bash ${BASE_DIR}/set_bashrc.sh ${BASE_DIR}

bash ${PARENT_DIR}/make.sh ${PARENT_DIR}
