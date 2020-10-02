<img src="https://raw.githubusercontent.com/wiki/PolySync/OSCC/images/oscc_logo_title.png">

# Kia soul commander [ ROS DASHING ] 

## Overview

Kia soul commander is an application designed to show how the Open Source Car Control API can be used to receive reports from and send commands to a drive by-wire enabled vehicle using ROS.

This application also demonstrates registering callback functions to recieve and parse OSCC reports as well as vehicle state reports from the car's OBD-II CAN network.

For more information about OSCC, check out PolySync OSCC on [github](https://github.com/PolySync/oscc).


## Getting Started

These instructions have been tested with a new Ubuntu 18.04 installation with ROS Dashing.

- OSCC's API and firmware modules are both required, and the modules must be installed on the vehicle
- The socketcan driver for USB and PCIe CAN interfaces is required, and is pre-installed on most Linux systems
- A CAN interface adapter, such as the [Kvaser Leaf Light](https://www.kvaser.com), is also necessary in order to connect the API to the OSCC control CAN network via USB

### 1. Set up this repository locally

Clone this repository to your machine, if you have not already done so:

```sh
# clone the repository
$ git clone git@github.com:irh-ca-team-car/oscc-commander.git
```
Note: If you do not have a SSH key on your github account you will need to clone the repository using HTTPS.

Then, from within that directory, sync and initialize Git submodules:

```sh
$ cd oscc-commander
$ git submodule sync
$ git submodule update --init --recursive
```

## Building Kia soul Commander

### 1. CMake Configuration

In order to build the oscc-commander application, you must use colcon. Place the directory inside a Colcon workspace

```sh
$ colcon build
$ source install/setup.bash
```

### CAN interface

You would then run the following:

```sh
#Root is important because the node change the can interface speed
su root
source /opt/ros/dashing/setup.bash
ros2 run kia_soul_control drivekit [channel=0]
```

For more information on setting up a socketcan interface, check out [this guide](http://elinux.org/Bringing_CAN_interface_up).

# Application Details

## Controlling the Vehicle with ROS

Once the kia soul commander is up and running you can use it to send commands to DriveKit via ROS topics.

### main

`main.cpp` is the entry point of oscc commander. Initializes OSCC interface, checks for ros topics updates every millisecond, and closes the interface when the program terminates. This contains the applications main loop.

### commander

`commander.cpp` The commander files contain the joystick commander's interactivity with the OSCC API. It demonstrates opening and closing the CAN channel communications with OSCC's control CAN network, sending enable/disable commands to the modules through the API, retrieving OSCC reports through callback functions, and sending commands through the OSCC `publish` functions.

`node.cpp` is the ROS2 node containing the commander update implementation.

`oscc.cpp` is a C++ version os `oscc.c` in the submodule oscc

# ROS

The current project references ros melodic in an non-standard way. The canbus is published under the can_msgs/Frame found in https://github.com/ros-industrial/ros_canopen. 

| topic_type             | topic_name                | direction |
| ---------------------- | ------------------------- | --------- |
| std_msgs::msg::Float64 | car/steering/torque       | Input     |
| std_msgs::msg::Float64 | car/throttle              | Input     |
| std_msgs::msg::Float64 | car/brake                 | Input     |
| std_msgs::msg::Bool    | car/enabled               | Input     |
| can_msgs::msg::Frame   | car/can0                  | Output    |
| std_msgs::msg::Float64 | car/speed/actual          | Output    |
| std_msgs::msg::Float64 | car/steering/angle/actual | Output    |

# Using OSCC API

To use the OSCC API in your applications, you need to include any relevant header files.

* The can message protocols are located in `oscc/api/include/can_protocols/`
    * These specify the structs we use for steering, throttle, brake, and fault reports
* Vehicle specific macros and values are located in `oscc/api/include/vehicles/`
	* You only need to include `vehicles.h`, which will include the relevant vehicle-specific header depending on the option provided to CMake (e.g., `-DVEHICLE=kia_soul_ev` will include `kia_soul_ev.h`)
* `oscc/api/include/oscc.h` includes the functionality to interface with the OSCC API


# License Information

MIT License

Copyright (c) 2020 PolySync Technologies

Please see [LICENSE.md](LICENSE.md) for more details.


# Contact Information

Please direct questions regarding OSCC and/or licensing to help@polysync.io.

*Distributed as-is; no warranty is given.*

Copyright (c) 2020 PolySync Technologies, Inc.  All Rights Reserved.
