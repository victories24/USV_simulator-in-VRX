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

### 一、安装基础环境

1.按照官方文档安装*ROS 2 Jazzy* 和 *Gazebo Harmonic*:

- [<u>ROS 2 Jazzy</u>](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [<u>Gazebo Harmonic</u>](https://gazebosim.org/docs/harmonic/install_ubuntu/)

2.安装依赖：

```bash
sudo apt install python3-sdformat14 ros-jazzy-xacro
```

### 二、安装VRX

1.创建colcon工作空间并克隆VRX仓库

```bash
mkdir -p ~/vrx_ws/src
cd ~/vrx_ws/src
git clone https://github.com/osrf/vrx.git
```

2.加载ROS2环境

```bash
source /opt/ros/jazzy/setup.bash
```

3.构建工作空间

```bash
cd ~/vrx_ws
colcon build --merge-install
```

4.配置运行环境

构建完成后，每次运行仿真前，都要加载 `setup.bash` 脚本。在根目录执行：

```bash
. install/setup.bash
```

（可选）为避免每次启动都要加载，可以在 `~/.bashrc` 文件(隐藏文件)中添加：

```bash
source ~/vrx_ws/install/setup.bash
```

5.尝试运行

第一次运行时，会同步从 [vrx collection on Fuel](https://app.gazebosim.org/OpenRobotics/fuel/collections/vrx) 下载3D模型，可能需要等待一段时间。

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

![VRX 仿真环境](picture/sydney_regatta.png)





# station_keeping
# way_finding
# path_following
