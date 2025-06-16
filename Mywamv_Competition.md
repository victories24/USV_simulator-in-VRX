# WAM-V的运动与规划
本片介绍了 `my_wamv` 中实现的小船自主运动，并在VRX提供的 [VRX Competition](https://github.com/osrf/vrx/wiki/vrx_2023-task_tutorials) 比赛任务中得到验证。

本文将从逆运动学控制，姿态控制，路径跟踪等方面对 `my_wamv` 包实现的功能进行解析。参考本文的方法可以进一步开发在VRX环境下的小船控制算法。



## 逆运动学控制

在VRX官方提供的教程中，WAM-V的操控需要在终端中一一发布左、右桨和舵的对应topic。在实现更进一步的功能之前，我们需要将小船的底层控制整合起来。将期望的运动速度指令转换为两个推进器的推力和角度指令，以便于高层的控制算法专注于路径规划或运动控制策略，而不必关心底层推进器的具体实现细节。

### 小船逆运动学理论

对一个双推进器的小船来说，每个推进器对小船施加的力可以沿x、y轴方向分解：

```math
\begin{cases}
F_{1,x} = F_1 \cdot \cos\theta_1 \\
F_{1,y} = F_1 \cdot \sin\theta_1 \\
F_{2,x} = F_2 \cdot \cos\theta_2 \\
F_{2,y} = F_2 \cdot \sin\theta_2
\end{cases}
```

<br>其中， $$F_1$$ 、 $$F_2$$ 分别为推进器推力； $$\theta_1$$ 、 $$\theta_2$$ 分别为推进器偏角。

参照以下公式建立动力学方程：

```math
\begin{cases}
F_x = \sum_{i=1}^2 F_{i,x} = F_1 \cos\theta_1 + F_2 \cos\theta_2 \\[8pt]
F_y = \sum_{i=1}^2 F_{i,y} = F_1 \sin\theta_1 + F_2 \sin\theta_2 \\[8pt]
M = \sum_{i=1}^2 (x_i F_{i,y} - y_i F_{i,x}) 
\end{cases}
```

<br>其中， $$F_x$$ 、 $$F_y$$ 分别为小船在x、y方向受到两推进器的力； $$M$$ 为小船的转矩。

在输入期望的运动速度指令（包含角速度与线速度）后，可以将 $$F_x$$ 、 $$F_y$$ 和 $$M$$ 看作已知量， $$F_{1,x}$$ 、 $$F_{1,y}$$ 、 $$F_{2,x}$$ 、 $$F_{2,y}$$ 为未知量，列出矩阵方程：

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



## station_keeping
## way_finding
## path_following




## 项目整合
