# USV_simulator-in-VRX

# VRX详解  

VRX(Virtual RobotX)是一个无人船的仿真环境，与RobotX比赛合作提供了水面无人船的仿真环境和任务，以及WAM-V平台。该项目模拟了真实的海洋场景，包括基于Fossen的波浪、洋流和风场模型；也搭建了全面的无人船平台，并提供了Lidar、摄像头、GPS和IMU等组件。

*下文将从安装、基础操作和深度解析三个方面对vrx进行解析。*
官方教程可参考[VRX Tutorial](https://github.com/osrf/vrx/wiki/tutorials)。

## 安装

官方教程[VRX Getting Started](https://github.com/osrf/vrx/wiki/getting_started_tutorial)。

本文的操作环境如下。新安装建议采用最新Release推荐版本*Gazebo Harmonic* 和 *ROS 2 Jazzy*。
```
ubuntu 22.04.5
ROS2 Humble
gazebo garden v7.9.0
vrx v2.4.1
```

### 1.安装基础环境

按照官方文档安装*ROS 2 Jazzy* 和 *Gazebo Harmonic*:

- [<u>ROS 2 Jazzy</u>](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [<u>Gazebo Harmonic</u>](https://gazebosim.org/docs/harmonic/install_ubuntu/)

# station_keeping
# way_finding
# path_following
