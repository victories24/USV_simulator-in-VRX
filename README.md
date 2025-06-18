# USV_Simulator-in-VRX

本项目实现了VRX环境下的WAM-V无人船仿真和运动规划。在完成WAM-V基础的操纵外，还编写了小船的逆运动学代码，在此基础上完成了**姿态保持**、**路径规划**和**轨迹跟踪**等功能，并通过VRX Competion提供的任务进行了验证。

项目的环境配置如下：

```bash
ubuntu 22.04.5
ROS2 Humble
gazebo garden v7.9.0
vrx v2.4.1
```

具体的项目实现细节分以下文档进行陈述：

- VRX仿真环境教学： [VRX_Tutorial](./VRX_Tutorial.md)
- WAM-V小船运动控制项目详解： [Mywamv_Competition.md](./Mywamv_Competition.md)

在此基础上，可以进一步开发无人船的寻路算法和跟踪算法等，并尝试通过PX4控制器向仿真环境输入GPS和IMU感应器数据。
