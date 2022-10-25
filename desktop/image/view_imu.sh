#!/usr/bin/env bash

set -e

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link &

rviz2 ./test_imu.rviz

