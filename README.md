## PincerRover – Ackermann Drive Rover with 4-DOF Arm (ROS2)

  This repository contains a ROS2-based control framework for an Ackermann-drive mobile robot equipped with a 4-DOF servo-driven arm. The system uses a HiWonder motor driver with integrated encoders, a PCA9685 servo controller, an MPU6886 IMU, and MG996R servos.
  The goal of this project is to provide a complete open-source ROS2 control stack for a compact rover platform that includes motion control, odometry, steering, IMU integration, and a lightweight manipulator.

## Hardware Overview

  Ackermann Rover Base
  HiWonder Motor Driver V1.4 (I2C address 0x34)
  JGB37-520 12V 110RPM motors with built-in hall encoders
  Encoder readout via 4 × 4-byte signed counters
  PCA9685 PWM board for steering servo
  MG996R steering servo
  12V battery supply with buck converters

4-DOF Arm

  MG996R servos for base, shoulder, elbow
  MG996R or SG90 for gripper
  Controlled using PCA9685 PWM channels

Sensors

  MPU6886 IMU for orientation and yaw estimation
  Integrated motor encoders for odometry
  Steering servo angle for Ackermann geometry

## Software Architecture

  ackermann_drive package includes:
  drive_odom.py: Main node for motor control and odometry
  drive_node.py: Entry point for the drive controller
  arm_controller.py: Node for controlling the 4-DOF arm
  imu_node.py: IMU publisher node
  launch directory for system launch files
  config directory for parameter tuning
  Ackermann Drive Node Features

## Capabilities:

  Controls motor speeds via HiWonder I2C interface
  Controls steering servo through PCA9685
  Computes odometry using encoder counts and IMU yaw
  Implements Ackermann steering geometry
  Maintains position, orientation, and velocity estimates
  Odometry Computation (summary):
  Linear velocity from encoder pulses
  Angular velocity from steering angle and wheelbase
  Uses differential kinematic model for Ackermann vehicles



