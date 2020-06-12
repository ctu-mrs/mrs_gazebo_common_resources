#!/bin/bash
# author: Robert Penicka
set -e

echo "Starting install preparation" 

openssl aes-256-cbc -K $encrypted_f2b1af48ae35_key -iv $encrypted_f2b1af48ae35_iv -in ./.ci/deploy_key_github.enc -out ./.ci/deploy_key_github -d
eval "$(ssh-agent -s)"
chmod 600 ./.ci/deploy_key_github
ssh-add ./.ci/deploy_key_github

sudo apt-get update -qq
sudo apt-get install dpkg git python-setuptools python3-setuptools python3-pip

echo "installing uav_core"
cd
git clone https://github.com/ctu-mrs/uav_core
cd uav_core
./installation/install.sh

echo "clone simulation"
cd
git clone https://github.com/ctu-mrs/simulation.git
cd simulation

echo "running the main install.sh"
./installation/install.sh

gitman update

# get the current commit SHA
cd "$TRAVIS_BUILD_DIR"
SHA=`git rev-parse HEAD`

# get the current package name
PACKAGE_NAME=${PWD##*/}

# checkout the SHA
cd ~/simulation/.gitman/$PACKAGE_NAME
git checkout "$SHA"

# will need this to test the compilation
sudo apt -y install python-catkin-tools

echo "creating workspace"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/simulation
source /opt/ros/melodic/setup.bash
cd ~/catkin_ws
catkin init

echo "install part ended"
