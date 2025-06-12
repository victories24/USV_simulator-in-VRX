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

- [VRX_Tutorial](./VRX_Tutorial.md)
- [Mywamv_Competition.md](./Mywamv_Competition.md)


