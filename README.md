# Immersive Embodied Telemanipulation System with a Velocity Controller

[![Releases](https://img.shields.io/github/release/Zhefan-Xu/CERLAB-UAV-Autonomy.svg)](https://github.com/RoboDD/usv_autonomy/releases)
![Noetic Ubuntu 20.04](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy/actions/workflows/Ubuntu20.04-build.yaml/badge.svg) 
[![license](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT) 

Coming Soon!

## 0. Demo

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/BAV0jQoAaEk/0.jpg)](https://www.youtube.com/embed/BAV0jQoAaEk)

## 1. Description of Files

- ```mr_teleop_lite6_mvc.py```: Cartesian velocity version.
<!-- - ```mr_teleop_lite6_mvj.py```: Joint velocity version. -->
<!-- - ```lite6.urdf```: URDF definitions for UFactory Lite 6 robotic manipulator. -->
<!-- - ```build_lite6.py```: imports Lite 6 URDF model -->
- ```lite6mr.exe```: Unity user client (PCVR). The source code of the Unity client might be released in a separate repo.

## 2. Hardware

- UFactory Lite 6 Robotic Manipulator (require velocity controller)
- Meta Quest 3 or Meta Quest Pro

## 3. Install

- Install Ubuntu 20.04 and ROS Noetic (recommend to use Windows+WSL2 [Ubuntu 20.04])
- Install `xarm_ros`: [link](https://github.com/xArm-Developer/xarm_ros)
- Install `ROS-TCP-Endpoint`: [link](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)
- Install dependencies: sm, rtb, r
<!-- - Copy this file to ```~/.local/lib/python3.8/site-packages/rtbdata/xacro``` -->
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
- Plug-in Quest Link, then run ```lite6mr.exe```. Reset initial view as desired.
- Open Ubuntu (WSL2), then run following nodes in separate terminals:

  ```bash
  ip addr show eth0 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1
  roslaunch ros_tcp_endpoint endpoint.launch
  roslaunch xarm_bringup lite6_server.launch robot_ip:=192.168.1.XXX
  rosrun xarm_bringup mr_teleop_lite6.py
  ```
- Enjoy!
  
## 5. Additional Setup for Agile Motion (Experimental)

May cause inevitable damage! Be very cautious! Be aware of safety!

- Turn on `reduced mode` for Lite 6
- Set joint speed limit to `180` deg/s
- Increase p_gain


## 6. Notes for adapting for other platforms

Let's build this codebase for diverse robots.

## 7. Citations

Immersive Embodied Telemanipulation System with a Velocity Controller [Under Review]
[paper](https://ieeexplore.ieee.org/)

## 8. Acknowledgments

The author would like to express his sincere gratitude to people from the Virtual Engineering Centre (VEC), XJTLU who contributed to the development of this research.

## 9. References

- unity robotics hub
- dkt
- hubotverse


## 10. Future Roadmap

- Diverse support for robotic platforms
- Enhanced UI
- Bimanual manipulation
