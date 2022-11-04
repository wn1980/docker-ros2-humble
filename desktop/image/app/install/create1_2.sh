#!/usr/bin/env bash

set -e

sudo apt update && sudo apt-get install -y ecl git

TURTLEBOT_WS=$HOME/dev_ws

if [ ! -d $TURTLEBOT_WS/src/create_robot ]; then

  mkdir -p $TURTLEBOT_WS/src

  cd $TURTLEBOT_WS/src

  git clone https://github.com/AutonomyLab/libcreate.git
  git clone https://github.com/AutonomyLab/create_robot.git -b foxy

fi

sudo apt-get update && sudo apt-get upgrade -y 

# make and install
cd $TURTLEBOT_WS

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
