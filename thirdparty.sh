#!/usr/bin/env bash

set -e

BASE_DIR="$(pwd)/thirdparty"

echo "Creating thirdparty directory for cloning"
mkdir -p "$BASE_DIR"

cd "$BASE_DIR"

echo "Cloning third-party repos"

git clone https://github.com/AprilRobotics/apriltag_ros.git

git clone -b jazzy https://github.com/ros-planning/moveit2.git

git clone -b jazzy https://github.com/ros-planning/navigation2.git

git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git

git clone -b jazzy-devel https://github.com/cra-ros-pkg/robot_localization.git

git clone -b jazzy https://github.com/SteveMacenski/slam_toolbox.git

git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git

git clone https://github.com/YDLIDAR/YDLidar-SDK.git

echo "Cloned successfully."
