# Immersive Embodied Telemanipulation Systems with Velocity Controllers

[![Releases](https://img.shields.io/github/release/Zhefan-Xu/CERLAB-UAV-Autonomy.svg)](https://github.com/RoboDD/usv_autonomy/releases)
![Noetic Ubuntu 20.04](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy/actions/workflows/Ubuntu20.04-build.yaml/badge.svg) 
[![license](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT) 
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Linux platform](https://img.shields.io/badge/platform-linux--arm-brown.svg)](https://releases.ubuntu.com/20.04/)

Coming soon!

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/1dyqM8UEa1E/0.jpg)](https://www.youtube.com/embed/1dyqM8UEa1E)

## 1. Description of Packages

- ```mr_teleop_lite6_mvj.py```: Main node.
- ```lite6.urdf```: URDF definitions for UFactory Lite 6 robotic manipulator.
- ```lite6mr.exe```: Unity user client.

## 2. Install

- Install Ubuntu 20.04 and ROS Noetic (recommend to use Windows+WSL2 [Ubuntu 20.04])
- Install `xarm_ros`: [link](https://github.com/xArm-Developer/xarm_ros)
- Install `ROS-TCP-Endpoint`: [link](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)

## 3. Bring-ups

```bash
roslaunch ros_tcp_endpoint endpoint.launch
roslaunch xarm_bringup lite6_server.launch robot_ip:=192.168.1.242
rosrun xarm_bringup mr_teleop_lite6_mvj.py
```

## 4. Citatations

## 5. Acknowledgement
