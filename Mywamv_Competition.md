# WAM-V的运动控制与规划
本片介绍了 `my_wamv` 中实现的小船自主运动，并在VRX提供的 [VRX Competition](https://github.com/osrf/vrx/wiki/vrx_2023-task_tutorials) 比赛任务中得到验证。

本文将从逆运动学控制，姿态控制，路径跟踪等方面对 `my_wamv` 包实现的功能进行解析。参考本文的方法可以进一步开发在VRX环境下的小船控制算法。<br><br>



## 逆运动学控制

在VRX官方提供的教程中，WAM-V的操控需要在终端中一一发布左、右桨和舵的对应topic。在实现更进一步的功能之前，我们需要将小船的底层控制整合起来。将期望的运动速度指令转换为两个推进器的推力和角度指令，以便于高层的控制算法专注于路径规划或运动控制策略，而不必关心底层推进器的具体实现细节。

### 一、逆运动学理论推导

对一个双推进器的小船来说，每个推进器对小船施加的力可以沿x、y轴方向分解：

```math
\begin{cases}
F_{1,x} = F_1 \cdot \cos\theta_1 \\
F_{1,y} = F_1 \cdot \sin\theta_1 \\
F_{2,x} = F_2 \cdot \cos\theta_2 \\
F_{2,y} = F_2 \cdot \sin\theta_2
\end{cases}
```

<br>其中， $F_1$ 、 $F_2$ 分别为推进器推力； $\theta_1$ 、 $\theta_2$ 分别为推进器偏角。

参照以下公式建立动力学方程：

```math
\begin{cases}
F_x = \sum_{i=1}^2 F_{i,x} = F_1 \cos\theta_1 + F_2 \cos\theta_2 \\[8pt]
F_y = \sum_{i=1}^2 F_{i,y} = F_1 \sin\theta_1 + F_2 \sin\theta_2 \\[8pt]
M = \sum_{i=1}^2 (x_i F_{i,y} - y_i F_{i,x}) 
\end{cases}
```

<br>其中， $F_x$ 、 $F_y$ 分别为小船在x、y方向受到两推进器的总推力； $M$ 为小船的转矩。

在输入期望的运动速度指令（包含角速度与线速度）后，可以求出所需施加的推力和转矩，满足非线性动力学方程：

```math
\boldsymbol{\nu} = \begin{bmatrix}
V_x \\
V_y \\
\omega_z
\end{bmatrix}, \quad
\boldsymbol{\tau} = \begin{bmatrix}
F_x \\
F_y \\
M
\end{bmatrix} 
```

```math
\boldsymbol{M}\dot{\boldsymbol{\nu}} + \boldsymbol{C}(\boldsymbol{\nu})\boldsymbol{\nu} + \boldsymbol{D}\boldsymbol{\nu} = \boldsymbol{\tau}
```

<br>其中：
- $\boldsymbol{M}$	：系统惯性矩阵（刚体+附加质量）
- $\boldsymbol{C}(\boldsymbol{\nu})$	：科里奥利-向心力矩阵
- $\boldsymbol{D}$	：阻尼矩阵

将 $F_x$ 、 $F_y$ 和 $M$ 看作已知量， $F_{1,x}$ 、 $F_{1,y}$ 、 $F_{2,x}$ 、 $F_{2,y}$ 为未知量，列出矩阵方程：

```math
\begin{bmatrix}
1 & 0 & 1 & 0 \\
0 & 1 & 0 & 1 \\
-y_1 & x_1 & -y_2 & x_2 
\end{bmatrix}
\begin{bmatrix}
F_{1,x} \\
F_{1,y} \\
F_{2,x} \\
F_{2,y} 
\end{bmatrix}
=
\begin{bmatrix}
F_x \\
F_y \\
M
\end{bmatrix}
```

<br>并添加约束：已知两推进器以x轴对称分布；求最小二范数解以保证最小功率消耗。

```math
\quad \text{s.t.} \quad
\begin{cases}
x_1 = x_2 = -x_{\text{offset}} \\
y_1 = -y_2 = y_{\text{offset}} \\
\|F\|_2^2 = F_{1,x}^2 + F_{1,y}^2 + F_{2,x}^2 + F_{2,y}^2 \to \min
\end{cases}
```

<br>最终求得解析解为：
```math
\begin{cases}
F_{1,x}^* = \dfrac{F_x}{2} - \dfrac{x_{\text{offset}}}{2y_{\text{offset}}}F_y - \dfrac{M_z}{2y_{\text{offset}}} \\[10pt]
F_{1,y}^* = \dfrac{F_y}{2} \\[10pt]
F_{2,x}^* = \dfrac{F_x}{2} + \dfrac{x_{\text{offset}}}{2y_{\text{offset}}}F_y + \dfrac{M_z}{2y_{\text{offset}}} \\[10pt]
F_{2,y}^* = \dfrac{F_y}{2}
\end{cases}
```

<br>进而推得每个推进器的推力大小和方向：

```math
\begin{aligned}
F_i = \sqrt{F_{i,x}^2 + F_{i,y}^2} \quad (i=1,2) \\
\theta_i = \arctan\left(\frac{F_{i,y}}{F_{i,x}}\right) \quad (i=1,2)
\end{aligned}
```

<br>至此，我们完成了从输入的运动速度指令 $V_x$ 、 $V_y$ 、 $\omega_z$ 到输出推进器推力和角度 $F_i$ 、 $\theta_i$ 的理论推导过程。



### 二、逆运动学控制算法实现

参照上文提供的逆运动学算法，本篇用python实现了无人船的逆运动学控制代码，参考 [mywamv_inverse_kinematics.py](./my_wamv/mywamv_inverse_kinematics.py)

1. **输入与输出**

输入为ROS `Twist` 消息 `/wamv/cmd_vel`，而输出为两个推进器的推力和角度topic，参考 [VRX教程](./VRX_Tutorial.md)

```python
# 订阅速度指令话题
self.cmd_vel_sub = self.create_subscription(
    Twist,
    '/wamv/cmd_vel',
    self.inverse_kinematics_callback,
    10
)

# 创建推进器指令发布者
self.left_thrust_pub = self.create_publisher(
    Float64,
    '/wamv/thrusters/left/thrust',
    10
)
self.right_thrust_pub = self.create_publisher(
    Float64,
    '/wamv/thrusters/right/thrust',
    10
)
self.left_pos_pub = self.create_publisher(
    Float64,
    '/wamv/thrusters/left/pos',
    10
)
self.right_pos_pub = self.create_publisher(
    Float64,
    '/wamv/thrusters/right/pos',
    10
)
```

2. **Twist消息到小船转矩的转换**

本代码将非线性动力学方程简化为线性关系，系数满足在范围内大致吻合：

```python
# 限制给予的线速度和角速度，保证不会因为过大的输入导致输出异常(前进或转向推力被覆盖)
msg.linear.x = constrain(msg.linear.x, -max_linear, max_linear)
msg.linear.y = constrain(msg.linear.y, -max_linear, max_linear)
msg.angular.z = constrain(msg.angular.z, -max_angular, max_angular)

# 计算期望的总力和力矩
Fx = v_x_gain * msg.linear.x
Fy = v_y_gain * msg.linear.y
Mz = w_z_gain * msg.angular.z
```

如果想要遵从方程准确计算合力与转矩， $\boldsymbol{M}$ 、 $\boldsymbol{C}(\boldsymbol{\nu})$ 和 $\boldsymbol{D}$ 的数据可以在VRX阻力插件 `libSimpleHydrodynamics.so` 中找到。

3. **计算推进器推力与角度**

参照方程计算推力与角度，同时还应注意旋转180度时转换为负推力，将角度限制在 $[-90^\circ, 90^\circ]$ 之间：

```python
# 计算左推进器分力 (F1x, F1y)，设左右点坐标为(-x,y)(-x,-y)
x = abs(left_x)
y = abs(left_y)
F1x = 0.5 * Fx - x / (2 * y) * Fy - Mz / (2 * y) 
F1y = 0.5 * Fy

# 计算右推进器分力 (F2x, F2y)
F2x = 0.5 * Fx + x / (2 * y) * Fy + Mz / (2 * y)
F2y = 0.5 * Fy

# 计算每个推进器的总推力和角度
left_thrust, left_angle = calculate_thrust_and_angle(F1x, F1y)
right_thrust, right_angle = calculate_thrust_and_angle(F2x, F2y)
```

*4. **限制角度变化率**

在现实场景中，小船的舵并不能同仿真环境中一样瞬时到达指定角度，而是以一定的速率缓慢转动到该角度。因此，可以在控制代码中加入对角度变化率的限制，以更真实地模拟实际情况。
根据需求增减这段代码：

```python
# 应用角速度限制（模拟实际桨匀速转向）
current_time = self.get_clock().now()
dt = (current_time - self.last_time).nanoseconds / 1e9
self.last_time = current_time

if dt > 0.0:  # 避免除以零
    # 计算最大允许角度变化
    max_delta_angle = max_angle_rate * dt
    
    # 限制左桨角度变化率
    delta_left = left_angle - self.last_left_angle
    if abs(delta_left) > max_delta_angle:
        left_angle = self.last_left_angle + math.copysign(max_delta_angle, delta_left)
    
    # 限制右桨角度变化率
    delta_right = right_angle - self.last_right_angle
    if abs(delta_right) > max_delta_angle:
        right_angle = self.last_right_angle + math.copysign(max_delta_angle, delta_right)
```


## station_keeping
## way_finding
## path_following




## 项目整合
