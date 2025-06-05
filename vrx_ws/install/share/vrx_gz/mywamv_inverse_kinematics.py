#!/usr/bin/env python3
# 两桨控制，基于x,y分力计算总推力和角度

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math
import numpy as np

def constrain(value, min_val, max_val):
    """约束数值在最小值和最大值之间"""
    return max(min_val, min(value, max_val))

def calculate_thrust_and_angle(Fx, Fy):
    """计算总推力和角度，并处理超过±90度的情况"""
    thrust = math.sqrt(Fx**2 + Fy**2)
    angle = math.atan2(Fy, Fx) if thrust > 0.01 else 0.0

    # 如果角度超过±90度，反转推力和角度
    if angle > math.pi/2:
        thrust *= -1
        angle -= math.pi
    elif angle < -math.pi/2:
        thrust *= -1
        angle += math.pi

    return thrust, angle

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('wamv_inverse_kinematics')
        
        # 声明参数并设置默认值
        self.declare_parameters(
            namespace='',
            parameters=[
                ('thrust_factor', 1.0),     # 反向推力补偿因子
                ('v_x_gain', 60.0),         # X轴线性速度增益
                ('v_y_gain', 60.0),         # Y轴线性速度增益
                ('w_z_gain', 200.0),         # Z轴角速度增益
                ('left_pos_x', -2.373776),   # 左桨x坐标
                ('left_pos_y', 1.027135),    # 左桨y坐标
                ('right_pos_x', -2.373776),  # 右桨x坐标
                ('right_pos_y', -1.027135),  # 右桨y坐标
                ('max_thrust', 200.0),      # 最大推力值
                ('max_angle', 3.1416),      # 最大转向角度(180度)
                ('debug', True)             # 调试模式开关
            ]
        )
        
        # 初始化推进器消息
        self.left_thrust_msg = Float64()
        self.right_thrust_msg = Float64()
        self.left_pos_msg = Float64()
        self.right_pos_msg = Float64()
        
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
        
        # 设置参数回调
        self.add_on_set_parameters_callback(self.config_callback)
        
        self.get_logger().info("WAM-V两桨逆运动学节点已初始化")

    def inverse_kinematics_callback(self, msg):
        # 获取当前参数值
        params = self.get_parameters([
            'thrust_factor', 'v_x_gain', 'v_y_gain', 'w_z_gain',
            'left_pos_x', 'left_pos_y', 'right_pos_x', 'right_pos_y',
            'max_thrust', 'max_angle', 'debug'
        ])
        
        thrust_factor = params[0].value
        v_x_gain = params[1].value
        v_y_gain = params[2].value
        w_z_gain = params[3].value
        left_x = params[4].value
        left_y = params[5].value
        right_x = params[6].value
        right_y = params[7].value
        max_thrust = params[8].value
        max_angle = params[9].value
        debug = params[10].value
        
        # 计算两桨之间的基线距离
        baseline = abs(left_y - right_y)
        
        if debug:
            self.get_logger().info(
                f"速度指令 - 线性X: {msg.linear.x:.4} m/s, "
                f"线性Y: {msg.linear.y:.4} m/s, "
                f"角速度Z: {msg.angular.z:.4} rad/s"
            )
        
        # 计算期望的总力和力矩
        Fx = v_x_gain * msg.linear.x
        Fy = v_y_gain * msg.linear.y
        Mz = w_z_gain * msg.angular.z
                
        # 计算每个推进器的x,y分力
        # 构建矩阵方程 Ax = b
        # [1   0   1   0  ] [F1x]   [Fx ]
        # [0   1   0   1  ] [F1y] = [Fy ]
        # [-ly lx -ry  rx ] [F2x]   [Mz ]
        # [0   0   0   0  ] [F2y]   [0  ]

        # 力平衡方程:
        # Fx = F1x + F2x
        # Fy = F1y + F2y
        # Mz = (F1y * left_x - F1x * left_y) + (F2y * right_x - F2x * right_y)

                
        # 计算左推进器分力 (F1x, F1y)，设左右点坐标为(-x,y)(-x,-y)
        x = abs(left_x)
        y = abs(left_y)
        F1x = 0.5 * Fx - x / (2 * y) * Fy - Mz / (2 * y) 
        F1y = 0.5 * Fy
        
        # 计算右推进器分力 (F2x, F2y)
        F2x = 0.5 * Fx + x / (2 * y) * Fy + Mz / (2 * y)
        F2y = 0.5 * Fy
        
        '''
        # 使用最小二乘法求解欠定方程组
        A = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [-left_y, left_x, -right_y, right_x],
            [0, 0, 0, 0]  # 伪行，保证矩阵形状
        ])
        b = np.array([Fx, Fy, Mz, 0])
        
        # 计算最小范数解
        try:
            x = np.linalg.lstsq(A, b, rcond=None)[0]
            F1x, F1y, F2x, F2y = x
        except np.linalg.LinAlgError:
            self.get_logger().error("矩阵求解失败，使用备用方案")
            F1x, F1y, F2x, F2y = Fx/2, Fy/2, Fx/2, Fy/2
        '''


        # 计算每个推进器的总推力和角度
        left_thrust, left_angle = calculate_thrust_and_angle(F1x, F1y)
        right_thrust, right_angle = calculate_thrust_and_angle(F2x, F2y)
        
        # 约束输出
        left_thrust = constrain(left_thrust, -max_thrust, max_thrust)
        right_thrust = constrain(right_thrust, -max_thrust, max_thrust)
        left_angle = constrain(left_angle, -max_angle, max_angle)
        right_angle = constrain(right_angle, -max_angle, max_angle)

        # 处理反向推力
        if left_thrust < 0.0:
            left_thrust*= thrust_factor
        if right_thrust < 0.0:
            right_thrust*= thrust_factor
        
        # 设置消息
        self.left_thrust_msg.data = left_thrust
        self.right_thrust_msg.data = right_thrust
        self.left_pos_msg.data = left_angle
        self.right_pos_msg.data = right_angle
        
        if debug:
            self.get_logger().info(
                "左推进器 - 推力: {:.2f}, 角度: {:.2f}°  "
                "右推进器 - 推力: {:.2f}, 角度: {:.2f}°".format(
                    left_thrust, math.degrees(left_angle),
                    right_thrust, math.degrees(right_angle)
                )
            )
        
        # 发布指令
        self.left_thrust_pub.publish(self.left_thrust_msg)
        self.right_thrust_pub.publish(self.right_thrust_msg)
        self.left_pos_pub.publish(self.left_pos_msg)
        self.right_pos_pub.publish(self.right_pos_msg)
    
    def config_callback(self, params):
        # 参数更新回调函数
        result = SetParametersResult(successful=True)
        for param in params:
            if param.name in ['v_x_gain', 'v_y_gain', 'w_z_gain']:
                if not (0.0 <= param.value <= 50.0):
                    result.successful = False
                    result.reason = f"{param.name}必须在0.0到50.0之间"
            elif param.name == 'max_thrust':
                if not (0.0 <= param.value <= 200.0):
                    result.successful = False
                    result.reason = "最大推力必须在0.0到200.0之间"
            elif param.name == 'max_angle':
                if not (0.0 <= param.value <= 1.5708):  # 0到90度
                    result.successful = False
                    result.reason = "最大角度必须在0.0到1.5708(90度)之间"
        
        if result.successful:
            self.get_logger().info("参数更新成功")
        else:
            self.get_logger().warn(result.reason)
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()