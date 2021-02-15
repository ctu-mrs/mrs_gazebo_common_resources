#!/bin/bash
set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting install"

# get the current commit SHA
SHA=`git rev-parse HEAD`

# get the current package name
PACKAGE_NAME=${PWD##*/}

sudo apt-get -y install git

echo "installing uav_core"
cd
git clone https://github.com/ctu-mrs/uav_core
cd uav_core
./installation/install.sh
gitman update

echo "installing simulation"
cd
git clone https://github.com/ctu-mrs/simulation
cd simulation
./installation/install.sh
gitman update

# checkout the SHA
cd ~/simulation/.gitman/$PACKAGE_NAME
git checkout "$SHA"

echo "creating workspace"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/simulation
ln -s ~/uav_core/ros_packages/mavros
ln -s ~/uav_core/ros_packages/mrs_msgs
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws
catkin init

echo "install part ended"
