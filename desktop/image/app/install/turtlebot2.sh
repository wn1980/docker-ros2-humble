#!/usr/bin/env bash

set -e

sudo apt update && sudo apt-get install -y git

TURTLEBOT_WS=$HOME/dev_ws

if [ ! -d $TURTLEBOT_WS/src/kobuki_ros ]; then

  mkdir -p $TURTLEBOT_WS/src

  cd $TURTLEBOT_WS/src

  #git clone https://github.com/stonier/sophus.git -b release/1.2.x
  #git clone https://github.com/stonier/ecl_core.git -b release/1.2.x
  #git clone https://github.com/stonier/ecl_lite.git -b release/1.1.x
  #git clone https://github.com/stonier/ecl_tools.git -b release/1.0.x

  #git clone https://github.com/CNURobotics/ecl_lite -b humble-test
  #git clone https://github.com/CNURobotics/ecl_core -b humble-test
  #git clone https://github.com/CNURobotics/kobuki_ros -b humble-test

  git clone https://github.com/stonier/sophus.git -b 1.3.1
  git clone https://github.com/stonier/ecl_core.git -b 1.2.1
  git clone https://github.com/stonier/ecl_lite.git -b 1.2.0
  git clone https://github.com/stonier/ecl_tools.git -b 1.0.3

  git clone https://github.com/kobuki-base/kobuki_core.git
  git clone https://github.com/kobuki-base/velocity_smoother.git
  git clone https://github.com/kobuki-base/cmd_vel_mux.git
  
  git clone https://github.com/kobuki-base/kobuki_ros_interfaces.git

  git clone https://github.com/kobuki-base/kobuki_ros.git

  # Additionals
  git clone https://github.com/wn1980/turtlebot2_ros2.git

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
