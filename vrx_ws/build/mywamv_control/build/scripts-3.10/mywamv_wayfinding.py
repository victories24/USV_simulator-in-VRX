#!/usr/bin/python3

''' 
增强版Wayfinding控制器 - 支持手动路径点更新
添加了路径点锁定机制和安全控制接口
'''

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
from std_srvs.srv import Empty, SetBool
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import pymap3d

class PIDController:
    def __init__(self, kP=1.0, kI=0.0, kD=0.0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.err_int = 0.0
        self.err_prev = 0.0
        self.t_prev = 0.0

    def control(self, err, t):
        dt = t - self.t_prev
        if dt <= 0:
            return 0.0
        
        if abs(err) < 2.0:  # Anti-windup
            self.err_int += err * dt
            self.err_int = np.clip(self.err_int, -10.0, 10.0)
        
        err_dif = (err - self.err_prev) / dt if dt > 0 else 0.0
        u = self.kP * err + self.kI * self.err_int + self.kD * err_dif
        return np.clip(u, -5.0, 5.0)

class Wayfinding(Node):
    def __init__(self):
        super().__init__('wayfinding')
        
        # ========== 参数配置 ==========
        self.origin = (-33.724223, 150.679736, 0.0)  # WGS84原点
        self.gps_offset = 0.85    # GPS天线偏移(m)
        self.goal_tol = 1.0       # 接近模式触发距离(m)
        self.pos_tol = 0.05       # 位置容差(m)
        self.rot_tol = 0.1       # 角度容差(rad)
        self.v_limit = 10.0        # 最大速度(m/s)
        self.v_const = 1.0        # 接近速度系数
        self.debug = True         # 调试模式

        # ========== 状态控制 ==========
        self.allow_auto_updates = False  # 是否允许自动更新路径点
        self.waypoint_lock = False       # 初始锁定状态，接受初始路径点后锁定
        self.cur_pos = None             # 当前位置 [x, y] (ENU)
        self.cur_rot = None             # 当前偏航角
        self.enu_waypoints = []         # ENU坐标系路径点
        self.wp_index = 0               # 当前路径点索引

        # ========== PID控制器 ==========
        self.pid_g2g = PIDController(kP=1.0, kI=0.1, kD=0.05)
        self.pid_sk_vx = PIDController(kP=0.3, kI=0.01, kD=0.0)
        self.pid_sk_vy = PIDController(kP=0.3, kI=0.01, kD=0.0)
        self.pid_sk_wz = PIDController(kP=0.5, kI=0.05, kD=0.1)

        # ========== ROS接口 ==========
        # 订阅
        self.gps_sub = self.create_subscription(
            NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.waypoint_sub = self.create_subscription(
            PoseArray, '/vrx/wayfinding/waypoints', self.waypoint_callback, 10)
        
        # 发布
        self.cmd_vel_pub = self.create_publisher(Twist, '/wamv/cmd_vel', 10)
        self.error_pub = self.create_publisher(Float32, '/wayfinding/pose_error', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 服务
        self.enable_updates_srv = self.create_service(
            SetBool, '/wayfinding/enable_updates', self.handle_enable_updates)
        self.update_waypoints_srv = self.create_service(
            Empty, '/wayfinding/update_waypoints', self.handle_update_waypoints)
        self.get_waypoints_srv = self.create_service(
            Empty, '/wayfinding/get_waypoints', self.handle_get_waypoints)
        
        # 定时器
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Wayfinding节点已启动，等待初始路径点...")

    # ========== 服务处理 ==========
    def handle_enable_updates(self, request, response):
        """启用/禁用路径点自动更新"""
        self.allow_auto_updates = request.data
        response.success = True
        response.message = f"路径点自动更新已{'启用' if request.data else '禁用'}"
        self.get_logger().info(response.message)
        return response

    def handle_update_waypoints(self, request, response):
        """准备接收新路径点"""
        self.waypoint_lock = False
        self.get_logger().info("已准备接收新路径点")
        return response

    def handle_get_waypoints(self, request, response):
        """获取当前路径点信息(日志输出)"""
        if not self.enu_waypoints:
            self.get_logger().info("当前没有有效路径点")
        else:
            for i, (x, y, yaw) in enumerate(self.enu_waypoints):
                self.get_logger().info(
                    f"路径点 {i+1}: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°")
        return response

    # ========== 核心功能 ==========
    def gps_to_enu(self, lat, lon):
        """GPS转ENU坐标系 (纬度, 经度) -> (x, y)"""
        return pymap3d.geodetic2enu(lat, lon, 0, *self.origin)[:2]

    def normalize_angle(self, angle):
        """角度归一化到[-π, π]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def quaternion_to_yaw(self, q):
        """四元数转偏航角"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def waypoint_callback(self, msg):
        """处理路径点更新"""
        if self.waypoint_lock and not self.allow_auto_updates:
            self.get_logger().debug("路径点已锁定，忽略自动更新")
            return
            
        # 安全检查
        if not msg.poses:
            self.get_logger().warn("收到空路径点消息，已忽略")
            return
            
        for pose in msg.poses:
            if not (-90 <= pose.position.x <= 90 and -180 <= pose.position.y <= 180):
                self.get_logger().error(f"非法坐标值: lat={pose.position.x}, lon={pose.position.y}")
                return

        # 处理新路径点
        self.enu_waypoints = []
        for pose in msg.poses:
            lat = pose.position.x
            lon = pose.position.y
            yaw = self.quaternion_to_yaw(pose.orientation)
            x, y = self.gps_to_enu(lat, lon)
            self.enu_waypoints.append([x, y, yaw])
        
        if self.enu_waypoints:
            self.wp_index = 0
            self.waypoint_lock = True  # 锁定新路径点
            self.get_logger().info(
                f"已更新 {len(self.enu_waypoints)} 个路径点 | "
                f"第一个点: x={self.enu_waypoints[0][0]:.2f}, y={self.enu_waypoints[0][1]:.2f}")
        else:
            self.get_logger().warning("路径点转换后为空列表！")
        self.waypoint_lock = True   # 接受一次数据后自动锁定

    def gps_callback(self, msg):
        """处理GPS数据"""
        x, y = self.gps_to_enu(msg.latitude, msg.longitude)
        
        # 补偿GPS天线偏移
        if self.cur_rot is not None:
            x += self.gps_offset * math.cos(self.cur_rot)
            y += self.gps_offset * math.sin(self.cur_rot)
        
        self.cur_pos = np.array([x, y])
        
        if self.debug:
            self.get_logger().info(
                f"GPS更新: 纬度={msg.latitude:.6f}, 经度={msg.longitude:.6f} | "
                f"ENU位置: x={x:.2f}, y={y:.2f}",
                throttle_duration_sec=1.0
            )

    def imu_callback(self, msg):
        """处理IMU数据"""
        self.cur_rot = self.quaternion_to_yaw(msg.orientation)
        if self.debug:
            self.get_logger().info(
                f"当前偏航角: {math.degrees(self.cur_rot):.1f}°",
                throttle_duration_sec=1.0
            )

    def publish_transforms(self):
        """发布TF坐标"""
        if self.cur_pos is None or self.cur_rot is None:
            return

        # WAMV当前位姿
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'wamv/base_link'
        t.transform.translation.x = float(self.cur_pos[0])
        t.transform.translation.y = float(self.cur_pos[1])
        t.transform.rotation.z = math.sin(self.cur_rot/2)
        t.transform.rotation.w = math.cos(self.cur_rot/2)
        self.tf_broadcaster.sendTransform(t)

        # 路径点可视化
        for i, (x, y, yaw) in enumerate(self.enu_waypoints):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = f'waypoint_{i}'
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.rotation.z = math.sin(yaw/2)
            t.transform.rotation.w = math.cos(yaw/2)
            self.tf_broadcaster.sendTransform(t)

    def control_loop(self):
        """主控制循环"""
        # 检查数据是否就绪
        if (self.cur_pos is None or self.cur_rot is None or 
            not self.enu_waypoints or self.wp_index >= len(self.enu_waypoints)):
            return

        self.publish_transforms()
        
        # 获取当前目标
        target_x, target_y, target_yaw = self.enu_waypoints[self.wp_index]
        target_pos = np.array([target_x, target_y])
        
        # 计算误差
        err_vec = target_pos - self.cur_pos
        distance = np.linalg.norm(err_vec)
        self.error_pub.publish(Float32(data=float(distance)))

        cmd_vel = Twist()
        t_now = self.get_clock().now().nanoseconds / 1e9

        # 计算机体坐标系分量
        body_x = math.cos(self.cur_rot)
        body_y = math.sin(self.cur_rot)
        along_err = err_vec[0] * body_x + err_vec[1] * body_y  # 前向误差
        cross_err = -err_vec[0] * body_y + err_vec[1] * body_x  # 侧向误差

        if distance > self.goal_tol:
            # 接近模式
            cmd_vel.linear.x = float(np.clip(self.v_const * along_err, -self.v_limit, self.v_limit))
            #cmd_vel.linear.y = float(np.clip(self.v_const * cross_err, -self.v_limit, self.v_limit))
            
            # 航向控制
            desired_heading = math.atan2(err_vec[1], err_vec[0])
            heading_err = self.normalize_angle(desired_heading - self.cur_rot)
            cmd_vel.angular.z = float(self.pid_g2g.control(heading_err, t_now))
        else:
            # 保持模式（精细控制）
            adaptive_gain = min(1.0, distance / self.goal_tol)
            cmd_vel.linear.x = float(self.pid_sk_vx.control(along_err * adaptive_gain, t_now))
            cmd_vel.linear.y = float(self.pid_sk_vy.control(cross_err * adaptive_gain, t_now))
            cmd_vel.angular.z = float(self.pid_sk_wz.control(
                self.normalize_angle(target_yaw - self.cur_rot), t_now
            ))

        self.cmd_vel_pub.publish(cmd_vel)

        # 检查是否到达当前路径点
        if (distance < self.pos_tol and 
            abs(self.normalize_angle(target_yaw - self.cur_rot)) < self.rot_tol):
            self.wp_index += 1
            if self.wp_index >= len(self.enu_waypoints):
                self.get_logger().info("已完成所有路径点！保持最后位置")
                self.wp_index = max(0, len(self.enu_waypoints) - 1)  # 保持在最后一个点
            else:
                self.get_logger().info(
                    f"到达路径点 {self.wp_index}/{len(self.enu_waypoints)} | "
                    f"下一个目标: x={self.enu_waypoints[self.wp_index][0]:.2f}, "
                    f"y={self.enu_waypoints[self.wp_index][1]:.2f}"
                )

        # 调试输出
        if self.debug:
            self.get_logger().info(
                f"目标点 {self.wp_index+1}/{len(self.enu_waypoints)} | "
                f"距离: {distance:.2f}m | "
                f"控制指令: Vx={cmd_vel.linear.x:.2f}, Vy={cmd_vel.linear.y:.2f}, Wz={cmd_vel.angular.z:.2f}",
                throttle_duration_sec=0.2
            )

def main(args=None):
    rclpy.init(args=args)
    node = Wayfinding()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被手动终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()