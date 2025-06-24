# USV_Simulator-in-VRX

本项目实现了VRX环境下的WAM-V无人船仿真和运动规划。在完成WAM-V基础的操纵外，还编写了小船的逆运动学代码，在此基础上完成了**姿态保持**、**路径规划**和**轨迹跟踪**等功能，并通过VRX Competion提供的任务进行了验证。

项目的环境配置如下：

```bash
ubuntu 22.04.5
ROS2 Humble
gazebo garden v7.9.0
vrx v2.4.1
```

- `vrx_ws` 包含整合的VRX环境工作包，参考 [VRX官方文档](https://github.com/osrf/vrx/wiki/tutorials)

- `my_wamv` 包含自建的无人船仿真任务配置和脚本，其中：
  - `config` ：无人船推进器和感应器配置文件
  - `urdf` ：无人船配置文件
  - `mywamv_control` ：项目整合工作包
  - `dubins_path_generator.py` ：为路径跟踪任务生成复杂路径
  - `figure_eight_generator.py` ：为路径跟踪任务生成八字型测试路径
  - `mywamv.launch.py` ：整合项目启动文件
  - `mywamv_keyboard_control.py` ：键盘操纵无人船运动
  - `mywamv_path_follow_adpLOS.py` ：自适应LOS算法实现路径跟踪
  - `mywamv_station_keeping.py` ：位姿控制任务
  - `mywamv_velocity.py` ：测速脚本
  - `mywamv_wayfinding.py` ：自动寻路任务
  - `rviz.launch.py` ：Rviz配置启动文件

具体的项目实现细节分以下文档进行陈述：

- VRX仿真环境教学： [VRX_Tutorial](./VRX_Tutorial.md)
- WAM-V小船运动控制项目详解： [Mywamv_Competition.md](./Mywamv_Competition.md)

在此基础上，可以进一步开发无人船的寻路算法和跟踪算法等，并尝试通过PX4控制器向仿真环境输入GPS和IMU感应器数据。
