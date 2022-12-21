#!/usr/bin/env bash

set -e

sudo apt update && sudo apt-get install -y git

sudo pip install odrive==0.5.4

DEV_WS=$HOME/dev_ws

if [ ! -d $DEV_WS/src/odrive_ros2_control ]; then

  mkdir -p $DEV_WS/src

  cd $DEV_WS/src

  git clone https://github.com/wn1980/odrive_ros2_control.git -b humble-fw-v0.5.3
  git clone https://github.com/ros-controls/ros2_control_demos.git
  mv ros2_control_demos/ros2_control_demo_description .
  rm -rf ros2_control_demos

fi

sudo apt-get update #&& sudo apt-get upgrade -y 

# make and install
cd $DEV_WS

sudo rosdep install -i --from-path src --rosdistro humble -y
  
source /opt/ros/${ROS_DISTRO}/setup.bash

colcon build --symlink-install

# Install additionals and clean
sudo apt-get install  -y \
  #ros-${ROS_DISTRO}-kobuki-core \
  #ros-${ROS_DISTRO}-kobuki-ftdi \
  #ros-${ROS_DISTRO}-kobuki-firmware \
  man \
  && sudo apt-get autoremove -y \
  && sudo apt-get clean \
  && sudo rm -rf /var/lib/apt/lists/*
