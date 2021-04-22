#!/bin/bash
set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

# get the path to this script
MY_PATH=`pwd`

echo "Starting install"

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

sudo apt-get update
sudo apt-get upgrade -y libignition-common3*

# link the up-to-date version of this package
rm -rf ~/simulation/.gitman/$PACKAGE_NAME
ln -s "$MY_PATH" ~/simulation/.gitman/$PACKAGE_NAME

echo "creating workspace"
mkdir -p ~/mrs_workspace/src
cd ~/mrs_workspace/src
ln -s ~/simulation
ln -s ~/uav_core/ros_packages/mavros
ln -s ~/uav_core/ros_packages/mrs_msgs
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/mrs_workspace
catkin init

echo "install part ended"
