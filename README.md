# HWT905 ROS driver

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

None

**Published Topics**

- /inclinometer/imu ([sensor_msgs/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html))
* The standard ROS imu sensor msg which include orientation by filtered RPY

**Sensor configuration**

Specific settings might be done via config/driver_config.yaml configuration file.

## 3. Running with docker

First of all, you need to clone this repository with (important) submodules:

```bash
git clone https://github.com/Ant-Robotics-Inc/hwt905-ros-driver --recursive
```

Don't forget to update submodules after each pull. Or call this command if you forget to add `--recursive` in previous command:

```bash
git submodule update --init --recursive
```

The easiest way to play with this package is to use [scripts/docker.sh](scripts/docker.sh) script.

Try `./scripts/docker.sh --help` to get detailed info.

Typically you need 3 commands:
- `./scripts/docker.sh build` to build the docker image
- `./scripts/docker.sh run` to run the docker container
- `./scripts/docker.sh kill` to kill all existed containers (just in case if your container unsuccessfully finished, sometimes it happens yet).

## 4. Running without docker

(in process)

## 5. Notes for developers

- [Official site](https://www.wit-motion.com/9-axis/witmotion-hwt905-rs232-high.html)
- [Official github repo](https://github.com/WITMOTION/HWT905-RS232)
- [HWT905 Inclinometer Use Instructions with PC video](https://youtu.be/yDCIK_ZQ0DE)
- [Datasheet](https://github.com/WITMOTION/HWT905-RS232/blob/master/HWT905%20RS232%20Datasheet.pdf)
- [User manual](https://github.com/WITMOTION/HWT905-RS232/blob/master/HWT905%20RS232%20Manual.pdf)
