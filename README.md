# Immersive Embodied Telemanipulation Systems with Velocity Controllers

[![Releases](https://img.shields.io/github/release/Zhefan-Xu/CERLAB-UAV-Autonomy.svg)](https://github.com/RoboDD/usv_autonomy/releases)
![Noetic Ubuntu 20.04](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy/actions/workflows/Ubuntu20.04-build.yaml/badge.svg) 
[![license](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT) 
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Linux platform](https://img.shields.io/badge/platform-linux--arm-brown.svg)](https://releases.ubuntu.com/20.04/)

Coming soon!

## 1. Description of Packages

- ```arm_bringup```: Nodes for initializing various components of the USV, such as the STM32 communication, IMU setup, and VIO integration.
- ```usv_description```: Configuration for visualizing the USV model in RViz, including URDF definitions and mesh files.
- ```usv_control```: Implementation of a PID controller for velocity control of the USV.
- ```usv_teleop```: Nodes and interfaces for teleoperating the USV using keyboard, joysticks, or VR interfaces.

## 2. Install


## 3. Demos

### 3.1. Simulation

### 3.2. Real-world

```bash
roslaunch ros_tcp_endpoint endpoint.launch
roslaunch xarm_bringup lite6_server.launch robot_ip:=192.168.1.242
rosrun xarm_bringup mr_teleop_lite6_mvj.py
```

## 4. Citatations

## 5. Acknowledgement
