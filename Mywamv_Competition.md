# WAM-V的运动与规划
本片介绍了 `my_wamv` 中实现的小船自主运动，并在VRX提供的 [VRX Competition](https://github.com/osrf/vrx/wiki/vrx_2023-task_tutorials) 比赛任务中得到验证。

本文将从逆运动学控制，姿态控制，路径跟踪等方面对 `my_wamv` 包实现的功能进行解析。参考本文的方法可以进一步开发在VRX环境下的小船控制算法。









## 逆运动学控制

在VRX官方提供的教程中，WAM-V的操控需要在终端中一一发布左、右桨和舵的对应topic。在实现更进一步的功能之前，我们需要将小船的底层控制整合起来。将期望的运动速度指令转换为两个推进器的推力和角度指令，以便于高层的控制算法专注于路径规划或运动控制策略，而不必关心底层推进器的具体实现细节。

### 小船逆运动学理论

对一个双推进器的小船来说，每个推进器对小船施加的力可以沿x、y轴方向分解：

```math
\begin{cases}
F_{total_x} = T_1 \cos\theta_1 + T_2 \cos\theta_2 \\
F_{total\_y} = T_1 \sin\theta_1 + T_2 \sin\theta_2 \\
\tau_{total} = (T_1 \sin\theta_1) l_x - (T_1 \cos\theta_1) l_y + (T_2 \sin\theta_2) r_x - (T_2 \cos\theta_2) r_y
\end{cases}
```






## station_keeping
## way_finding
## path_following




## 项目整合
