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

echo "clone simulation"
cd
git clone git@github.com:ctu-mrs/simulation.git
cd simulation

echo "running the main install.sh"
./installation/install.sh

gitman update

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/simulation
source /opt/ros/melodic/setup.bash
cd ~/catkin_ws
catkin init

echo "install part ended"
