#!/usr/bin/env python3

''' 传感器输入：
    GPS → gps_callback → 更新 cur_pos
    IMU → imu_callback → 更新 cur_rot

    目标输入：
    VRX任务 → goal_callback → 更新 cmd_pos 和 cmd_rot

    控制输出：
    control_loop → 计算误差 → PID控制 → 发布 cmd_vel
    同时发布 pose_error 供监控。'''

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
import pymap3d

class PIDController:
    def __init__(self, kP=1.0, kI=0.0, kD=0.0):
        self.kP = kP         # 比例增益
        self.kI = kI         # 积分增益
        self.kD = kD         # 微分增益
        self.err_int = 0.0   # 积分误差累积
        self.err_prev = 0.0  # 上一次的误差值
        self.t_prev = 0.0    # 上一次调用的时间戳

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 0:
            return 0.0
        
        # 积分抗饱和（仅在小误差时启用）
        if abs(err) < 2.0:
            self.err_int += err * dt
            self.err_int = np.clip(self.err_int, -10.0, 10.0)
        
        # 微分项
        err_dif = (err - self.err_prev) / dt if dt > 0 else 0.0
        
        # PID输出
        u = self.kP * err + self.kI * self.err_int + self.kD * err_dif
        u = np.clip(u, -5.0, 5.0)  # 通用输出限制
        
        self.err_prev = err
        self.t_prev = t
        return u

class StationKeeping(Node):
    def __init__(self):
        super().__init__('station_keeping')
        
        # 静态参数配置
        self.origin = (-33.724223, 150.679736, 0.0)  # WGS84原点
        self.gps_offset = 0.85  # GPS天线偏移（米）
        self.goal_tol = 1.0     # 接近模式触发距离（米）
        self.v_limit = 5.0      # GtG速度限制（米/秒）
        self.v_const = 0.5      # GtG接近阶段速度比例增益
        self.debug = True       # 调试模式

        # PID控制器（静态参数）
        self.pid_g2g = PIDController(kP=1.0, kI=0.1, kD=0.05)   # Go-to-goal angular
        self.pid_sk_vx = PIDController(kP=0.3, kI=0.01, kD=0.0) # Station-keeping Vx
        self.pid_sk_vy = PIDController(kP=0.3, kI=0.01, kD=0.0) # Station-keeping Vy
        self.pid_sk_wz = PIDController(kP=0.5, kI=0.05, kD=0.1) # Station-keeping Wz

        # 状态变量
        self.cur_pos = None  # [x, y]
        self.cur_rot = None  # yaw (rad)
        self.cmd_pos = None  # [x, y]
        self.cmd_rot = 0.0

        # ROS接口
        self.gps_sub = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/vrx/stationkeeping/goal', self.goal_callback, 10)
        self.pose_error_pub = self.create_publisher(Float32, '/station_keeping/pose_error', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/wamv/cmd_vel', 10)
        self.control_timer = self.create_timer(0.05, self.control_loop)

    def gps_to_enu(self, lat, lon, alt=0.0):
        """WGS84转ENU坐标系"""
        return pymap3d.geodetic2enu(lat, lon, alt, *self.origin)

    def normalize_angle(self, angle):
        """角度归一化到[-π, π]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def quaternion_to_yaw(self, q):
        """四元数转偏航角yaw"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg):
        """处理目标位置"""
        x, y, _ = self.gps_to_enu(msg.pose.position.x, msg.pose.position.y)
        self.cmd_pos = np.array([x, y])  # 存储ENU坐标系下的目标位置
        self.cmd_rot = self.quaternion_to_yaw(msg.pose.orientation)  # 存储目标偏航角

    def gps_callback(self, msg):
        """处理GPS数据"""
        x, y, _ = self.gps_to_enu(msg.latitude, msg.longitude)
        if self.cur_rot is not None:
            x += self.gps_offset * math.cos(self.cur_rot)
            y += self.gps_offset * math.sin(self.cur_rot)
        self.cur_pos = np.array([x, y])

    def imu_callback(self, msg):
        """处理IMU数据"""
        self.cur_rot = self.quaternion_to_yaw(msg.orientation)

    def control_loop(self):
        """主控制循环"""
        if self.cur_pos is None or self.cur_rot is None or self.cmd_pos is None:
            return
            
        err_vec = self.cmd_pos - self.cur_pos  # 位置误差向量 [Δx, Δy]
        distance = np.linalg.norm(err_vec)     # 到目标的欧氏距离
        self.pose_error_pub.publish(Float32(data=float(distance)))  # 发布误差

        cmd_vel = Twist()
        t_now = self.get_clock().now().nanoseconds

        if distance > self.goal_tol:
            # Go-to-goal模式（远距离接近）：控制Vx和Wz
            desired_heading = math.atan2(err_vec[1], err_vec[0])
            heading_error = self.normalize_angle(desired_heading - self.cur_rot)
            
            cmd_vel.linear.x = np.clip(self.v_const * distance, -self.v_limit, self.v_limit)
            cmd_vel.angular.z = self.pid_g2g.control(heading_error, t_now)
        else:
            # Station-keeping模式
            body_x = math.cos(self.cur_rot)
            body_y = math.sin(self.cur_rot)
            along_track = err_vec[0] * body_x + err_vec[1] * body_y
            cross_track = -err_vec[0] * body_y + err_vec[1] * body_x

            # 动态响应调整
            adaptive_gain = min(1.0, distance / self.goal_tol)
            along_track *= adaptive_gain
            cross_track *= adaptive_gain

            cmd_vel.linear.x = self.pid_sk_vx.control(along_track, t_now)
            cmd_vel.linear.y = self.pid_sk_vy.control(cross_track, t_now)
            cmd_vel.angular.z = self.pid_sk_wz.control(
                self.normalize_angle(self.cmd_rot - self.cur_rot), t_now
            )

        self.cmd_vel_pub.publish(cmd_vel)

        if self.debug:
            self.get_logger().info(
                f"Control: Vx={cmd_vel.linear.x:.2f}, Vy={cmd_vel.linear.y:.2f}, Wz={cmd_vel.angular.z:.2f} | "
                f"Dis_err={distance:.2f}m, Agu_err={self.normalize_angle(self.cmd_rot - self.cur_rot):.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = StationKeeping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()