#!/bin/bash

## Author: Michael
## Data: 2022-09-19
## Usage: ./ros_melodic_install.sh your_computer_password

password="$1"

## Setup your sources.list
echo 'password' | sudo -S sh -c 'echo "dep http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# # sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

## Setup your keys
sudo apt -y install curl
curl -s curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

## Installation
sudo apt update
sudo apt -y install ros-melodic-desktop-full

## Environment setup
rossource="source /opt/ros/melodic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in ~/.bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource

## Dependencies for building packages
echo 'password' | sudo -S apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update