# HWT905 ROS driver

[![Catkin build](https://github.com/Ant-Robotics-Inc/hwt905-ros-driver/workflows/catkin_build/badge.svg?branch=master)](https://github.com/Ant-Robotics-Inc/hwt905-ros-driver/actions/workflows/catkin_build.yml)

[WitMotion HWT905](https://www.wit-motion.com/9-axis/witmotion-hwt905-rs232-high.html) is RS232 High Accuracy 0.05 ° Military-Grade Sensor Inclinometer 9 Axis AHRS Sensor Waterproof IP67 & Anti-vibration.

This is ROS-package that allows you work with this device.

## Content
  - [1. Requirements](#1-requirements)
  - [2. ROS API](#2-ros-api)
  - [3. Running with docker](#3-running-with-docker)
  - [4. Running without docker](#4-running-without-docker)
  - [5. Notes for developers](#5-notes-for-developers)

## 1. Requirements

- ubuntu 18.04
- ros melodic

## 2. ROS API

**Subscribed Topics**

- None

**Published Topics**

- /inclinometer ([sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)) - the standard ROS imu sensor msg which include orientation, angular velocity and linear acceleration

**Sensor configuration**

Specific settings might be done via config/driver_config.yaml configuration file.

## 3. Running with docker

First of all, you need to clone this repository:

```bash
git clone https://github.com/Ant-Robotics-Inc/hwt905-ros-drivers
```

The easiest way to play with this package is to use [scripts/docker.sh](scripts/docker.sh) script.

Try `./scripts/docker.sh --help` to get detailed info.

Typically you need 5 commands:
- `./scripts/docker.sh build` to build the docker image
- `./scripts/docker.sh run` to run the docker container with inclinometer node
- `./scripts/docker.sh rviz` to run the docker container with inclinometer node + RVIZ node
- `./scripts/docker.sh interactive` to run the docker container in interactive mode
- `./scripts/docker.sh kill` to kill all existed containers (just in case if your container unsuccessfully finished, sometimes it happens yet).

## 4. Running without docker

(in process)

## 5. Notes for developers

- [Official site](https://www.wit-motion.com/9-axis/witmotion-hwt905-rs232-high.html)
- [Official github repo](https://github.com/WITMOTION/HWT905-RS232)
- [HWT905 Inclinometer Use Instructions with PC video](https://youtu.be/yDCIK_ZQ0DE)
- [Datasheet](https://github.com/WITMOTION/HWT905-RS232/blob/master/HWT905%20RS232%20Datasheet.pdf)
- [User manual](https://github.com/WITMOTION/HWT905-RS232/blob/master/HWT905%20RS232%20Manual.pdf)
