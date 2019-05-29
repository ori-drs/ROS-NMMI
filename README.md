!ROS node for NMMI device.

This repository contains the building blocks for ROS related applications based on _NMMI_ devices. It is composed by all the device-independent structures to interface our devices with the ROS ecosystem, therefore is barely usable as is.

## Table of Contents
1. [Installation](#markdown-header-installation)
   1. [Requirements](#markdown-header-requirements)
   1. [Sources](#markdown-header-sources)
   1. [Device Setup](#markdown-header-device-setup)
1. [Usage](#markdown-header-usage)
   1. [Communication Handler](#markdown-header-communication-handler)

## Installation
### Requirements
If you have never set it up, you probably need to add your linux user to the `dialout` group to grant right access to the serial port resources. To do so, just open a terminal and execute the following command:
```
sudo gpasswd -a <user_name> dialout
```
where you need to replace the `<user_name>` with your current linux username.

_Note: don't forget to logout or reboot._

```

### Sources
>Since you are interested in the ROS interfaces for our devices, it is assumed that you are familiar at least with the very basics of the ROS environment. If not, it might be useful to spend some of your time with [ROS](http://wiki.ros.org/ROS/Tutorials) and [catkin](http://wiki.ros.org/catkin/Tutorials) tutorials. After that, don't forget to come back here and start having fun with our Nodes.

Install the _NMMI device_ packages for a ROS user is straightforward. Nonetheless the following are the detailed steps which should be easy to understand even for ROS beginners:

1. Clone both the `ROS-NMMI` and `ROS-base` packages to your Catkin Workspace, e.g. `~/catkin_ws`:

1. Compile the packages using `catkin`:
   ```
   cd `~/catkin_ws`
   catkin_make
   ```
   **Note:** depending on your ROS installation, you may need some extra packages to properly compile the code. Please, be sure that you have already installed at least `ros-kinetic-ros-controllers`, `ros-kinetic-transmission-interface`, `ros-kinetic-joint-limits-interface`, and their dependencies (_e.g. use `sudo apt install <ros-pkg>`_).


### Device Setup
Connect a _NMMI device_ to your system is basically a matter of plugging in a USB cable. Nonetheless, **read carefully** the [manual](https://www.qbrobotics.com/products/qb-softhand-research/) to understand all the requirements and advices about either single-device or chained configurations.

## Usage
>Even if these ROS packages can be installed properly as a standalone application, they are barely usable alone. In addition to this package, please install also ROS-base package.

### Communication Handler
The Communication Handler Node has no parameters to be set, therefore it is always launched like this:
```
roslaunch nmmi_driver communication_handler.launch
```

On start, it scans the serial communication resources connected to your system and shows a list of the devices it has found. By default, it never scans again for new devices, apart from asking it explicitly during the initialization of a control Node.

This is a simple example when starting the Communication Handler with two _NMMI_ devices connected on two distinct USB ports:
```
[ INFO] [1524044523.511369300]: [CommunicationHandler] handles [/dev/ttyUSB1].
[ INFO] [1524044524.426984697]: [CommunicationHandler] handles [/dev/ttyUSB0].
[ INFO] [1524044525.218613760]: [CommunicationHandler] has found [2] devices connected:
[ INFO] [1524044525.218696997]:                        - device [1] connected through [/dev/ttyUSB0]
[ INFO] [1524044525.218736612]:                        - device [2] connected through [/dev/ttyUSB1]
```

When the Communication Handler is on, it provides all the Services required to interact with the connected devices: e.g. _get info or measurements, activate or deactivate motors, set commands_, and even more...