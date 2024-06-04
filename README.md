# Immersive Embodied Telemanipulation Systems with Velocity Controllers

[![Releases](https://img.shields.io/github/release/Zhefan-Xu/CERLAB-UAV-Autonomy.svg)](https://github.com/RoboDD/usv_autonomy/releases)
![Noetic Ubuntu 20.04](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy/actions/workflows/Ubuntu20.04-build.yaml/badge.svg) 
[![license](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT) 

Comming Soon!

## 0. Demo

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/1dyqM8UEa1E/0.jpg)](https://www.youtube.com/embed/1dyqM8UEa1E)

## 1. Description of Files

- ```mr_teleop_lite6_mvc.py```: Cartesian velocity version.
- ```mr_teleop_lite6_mvj.py```: Joint velocity version.
- ```lite6.urdf```: URDF definitions for UFactory Lite 6 robotic manipulator.
- ```lite6mr.exe```: Unity user client (PCVR).

## 2. Hardware

- UFactory Lite 6 Robotic Manipulator (require joint velocity controller)
- Meta Quest 3 or Meta Quest Pro

## 3. Install

- Install Ubuntu 20.04 and ROS Noetic (recommend to use Windows+WSL2 [Ubuntu 20.04])
- Install `xarm_ros`: [link](https://github.com/xArm-Developer/xarm_ros)
- Install `ROS-TCP-Endpoint`: [link](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)
- Install dependencies: sm, rtb, r
- Download Oculus App: [link](https://www.meta.com/gb/quest/setup/)
  - Version: Oculus App 60.0.0.162.352 (60.0.0.162.352) 
  - Connect your MR device to your PC, normally Windows, using Quest Link (recommended) or AirLink
  - Enable developer mode: [link](https://developer.oculus.com/documentation/native/android/mobile-device-setup/?locale=en_GB)
  - Oculus App -> Setting -> General ->Turn on "unknown source"
  - Oculus App -> Setting -> Beta -> Turn on "Runtime", "Oculus Link Passthrough (required)"
  - Launch Quest Link : [link](https://www.meta.com/en-gb/help/quest/articles/headsets-and-accessories/oculus-link/connect-link-with-quest-2/)
- Download ```lite6mr.exe``` from release page

## 4. Bring-up

- Open xArm Studio through browser: `http://192.168.1.XXX:18333`
- Open Ubuntu (WSL2), then run following nodes in separate terminals:

  ```bash
  roslaunch ros_tcp_endpoint endpoint.launch
  roslaunch xarm_bringup lite6_server.launch robot_ip:=192.168.1.XXX
  rosrun xarm_bringup mr_teleop_lite6_mvj.py
  ```

## 5. Additional Setup for Agile Motion (Experimental)

May cause inevitable damage! Be very cautious!

## 6. Call for Contributions

Let's build this codebase for diverse robots.

## 7. Citations

Immersive Embodied Telemanipulation Systems with Velocity Controllers [Under Review]
[paper](https://ieeexplore.ieee.org/)

## 8. Acknowledgments

The author would like to express his sincere gratitude to Prof. X and Y for his great support and all VEC team members who contribute to the development of this research.
