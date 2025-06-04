#!/usr/bin/env python3

"""
增强版路径跟踪控制器：
- 改进的LOS算法
- 曲率自适应前视距离
- 非线性侧滑角补偿
- 增强的收敛模式判断
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import pymap3d
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

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
        
        # 积分抗饱和
        if abs(err) < 2.0:
            self.err_int += err * dt
            self.err_int = np.clip(self.err_int, -10.0, 10.0)
        
        err_dif = (err - self.err_prev) / dt if dt > 0 else 0.0
        u = self.kP * err + self.kI * self.err_int + self.kD * err_dif
        u = np.clip(u, -5.0, 5.0)
        
        self.err_prev = err
        self.t_prev = t
        return u

    def reset_integral(self):
        self.err_int = 0.0

class EnhancedPathTrackingController(Node):
    def __init__(self):
        super().__init__('enhanced_path_tracking_controller')
        
        # 参数配置
        self.origin = (-33.724223, 150.679736, 0.0)
        self.gps_offset = 0.85
        self.lookahead_dist = 5.0 # 初始前视距离
        self.min_lookahead = 1.5
        self.max_lookahead = 30.0
        self.speed_limit = 2.5      
        self.beta_gain = 0.3       # 侧滑角补偿系数
        self.converge_dist = 5.0  # 降低收敛阈值
        self.max_angular_rate = math.radians(45)  # 降低最大角速度
        self.smooth_factor = 0.2   # 指令平滑系数

        # 控制状态
        self.converge_mode = True
        self.mode_lock_time = 0
        self.last_cmd = Twist()
        self.last_heading_error_sign = 0

        # PID控制器
        self.pid_vx = PIDController(kP=1.2, kI=0.05, kD=0.1)
        self.pid_wz = PIDController(kP=0.4, kI=0.01, kD=0.5)  

        # 状态变量
        self.cur_pos = None
        self.cur_rot = None
        self.beta = 0.0          # 侧滑角
        self.path_points = []
        self.path_tangents = []
        self.current_target_idx = 0
        self.current_lookahead = None  # 存储当前前视点

        # ROS接口
        self.gps_sub = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.path_sub = self.create_subscription(Path, '/planning/path', self.path_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/lookahead_marker', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/wamv/cmd_vel', 10)

        # TF广播器
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()  # 初始化时发布静态 TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_timer = self.create_timer(0.01, self.publish_markers)
        self.control_timer = self.create_timer(0.05, self.control_loop)

    def gps_to_enu(self, lat, lon, alt=0.0):
        """WGS84转ENU坐标系"""
        return pymap3d.geodetic2enu(lat, lon, alt, *self.origin)[:2]

    def normalize_angle(self, angle):
        """角度归一化到[-π, π]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def quaternion_to_yaw(self, q):
        """四元数转偏航角"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def path_callback(self, msg):
        """带连续性的路径更新"""
        if len(msg.poses) < 2:
            return
            
        # 保留当前最近点索引
        prev_idx = self.current_target_idx if self.path_points else 0
        
        # 处理新路径
        self.path_points = []
        self.path_tangents = []
        
        for i, pose in enumerate(msg.poses):
            x = pose.pose.position.x
            y = pose.pose.position.y
            self.path_points.append([x, y])
            
            # 计算切线方向
            if i == 0:
                dx = msg.poses[1].pose.position.x - x
                dy = msg.poses[1].pose.position.y - y
            else:
                dx = x - msg.poses[i-1].pose.position.x
                dy = y - msg.poses[i-1].pose.position.y
            
            norm = math.hypot(dx, dy)
            if norm > 1e-6:
                dx /= norm
                dy /= norm
            self.path_tangents.append([dx, dy])
        
        # 寻找最近点
        if self.path_points and self.cur_pos is not None:
            distances = [np.linalg.norm(np.array(p)-self.cur_pos) 
                        for p in self.path_points]
            self.current_target_idx = np.argmin(distances)
        
        # 路径更新时重置PID积分项
        self.pid_vx.reset_integral()
        self.pid_wz.reset_integral()
        
        # 模式切换保护
        current_time = self.get_clock().now().nanoseconds
        if current_time - self.mode_lock_time > 5e8:  # 0.5秒锁定期
            self.converge_mode = True
            self.mode_lock_time = current_time

    def gps_callback(self, msg):
        """将WGS84经纬度转换为ENU坐标"""
        x, y = self.gps_to_enu(msg.latitude, msg.longitude)
        
        # 补偿GPS天线偏移
        if self.cur_rot is not None:
            x += self.gps_offset * math.cos(self.cur_rot)
            y += self.gps_offset * math.sin(self.cur_rot)
        
        self.cur_pos = np.array([x, y])

    def imu_callback(self, msg):
        """改进的侧滑角测量方法"""
        self.cur_rot = self.quaternion_to_yaw(msg.orientation)
        
        linear_accel = np.array([msg.linear_acceleration.x, 
                            msg.linear_acceleration.y])
        accel_norm = np.linalg.norm(linear_accel)
        
        if accel_norm > 0.3:  # 保持固定阈值0.3
            #  基础测量
            accel_dir = math.atan2(linear_accel[1], linear_accel[0])
            raw_beta = self.normalize_angle(accel_dir - self.cur_rot)
            
            # 动态置信度加权（加速度越大置信度越高）
            accel_weight = min(1.0, (accel_norm - 0.3) / 0.7)  # 0.3-1.0m/s²区间线性加权
            
            # 改进的滤波策略
            if abs(raw_beta - self.beta) < math.radians(10):  # 小变化时正常更新
                self.beta = 0.8 * self.beta + 0.2 * raw_beta * accel_weight
            else:  # 大变化时强滤波
                self.beta = 0.9 * self.beta + 0.1 * raw_beta * accel_weight
                
            # 增加物理合理性约束
            max_beta = math.radians(30) * min(1.0, accel_norm/1.0)  # 加速度越大允许的β越大
            self.beta = np.clip(self.beta, -max_beta, max_beta)
        else:
            # 低加速度时更保守的衰减
            self.beta *= 0.9  # 原为0.95
            
        # 最终输出限制（保持与原有接口一致）
        self.beta = np.clip(self.beta, -math.radians(30), math.radians(30))

    def estimate_path_curvature(self, window_size=3):
        """估计当前路径段的曲率"""
        if len(self.path_points) < window_size or self.current_target_idx < 1:
            return 0.0
        
        start_idx = max(0, self.current_target_idx - window_size)
        end_idx = min(len(self.path_points)-1, self.current_target_idx + window_size)
        
        points = self.path_points[start_idx:end_idx+1]
        if len(points) < 3:
            return 0.0
        
        # 使用三点法计算曲率
        p1, p2, p3 = points[0], points[len(points)//2], points[-1]
        
        # 计算三角形面积
        area = 0.5 * abs(
            (p2[0]-p1[0])*(p3[1]-p1[1]) - (p3[0]-p1[0])*(p2[1]-p1[1])
        )
        
        # 计算边长
        a = np.linalg.norm(np.array(p1)-np.array(p2))
        b = np.linalg.norm(np.array(p2)-np.array(p3))
        c = np.linalg.norm(np.array(p3)-np.array(p1))
        
        # 计算曲率 (4*面积/(边长乘积))
        curvature = 4 * area / (a * b * c + 1e-6)
        
        return min(curvature, 0.5)  # 限制最大曲率

    def find_lookahead_point(self):
        """基于LOS方法寻找路径上的目标点"""
        if not self.path_points or self.cur_pos is None:
            return None, None

        # 从当前目标点开始搜索
        closest_dist = float('inf')
        closest_idx = 0
        lookahead_point = None
        tangent = None
        accumulated_dist = 0.0
    
        # 首先找到最近点
        for i in range(len(self.path_points)):
            point = np.array(self.path_points[i])
            dist = np.linalg.norm(point - self.cur_pos)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i
        
        # 从最近点开始计算沿路径距离
        prev_point = np.array(self.path_points[closest_idx])
        for i in range(closest_idx, len(self.path_points)):
            current_point = np.array(self.path_points[i])
            segment_dist = np.linalg.norm(current_point - prev_point)
            accumulated_dist += segment_dist
            
            if accumulated_dist >= self.lookahead_dist:
                lookahead_point = current_point
                tangent = np.array(self.path_tangents[i])
                self.current_target_idx = i
                break
            
            prev_point = current_point
        
        # 如果未找到，返回最后一个点和切线
        if lookahead_point is None:
            lookahead_point = np.array(self.path_points[-1])
            tangent = np.array(self.path_tangents[-1])
            self.current_target_idx = len(self.path_points) - 1
            
        return lookahead_point, tangent

    def compute_cross_track_error(self, lookahead_point, tangent):
        """计算横向跟踪误差"""
        if lookahead_point is None or tangent is None:
            return 0.0
        
        # 计算路径法向量
        normal = np.array([-tangent[1], tangent[0]])
        
        # 计算横向误差
        error_vec = self.cur_pos - lookahead_point
        cross_track_error = np.dot(error_vec, normal)
        
        return cross_track_error

    def adaptive_lookahead(self, cross_track_error, speed=1.0):
        """改进的自适应前视距离"""
        path_curvature = self.estimate_path_curvature()
        
        # 基础前视距离考虑速度和曲率
        base_lookahead = self.min_lookahead + 3.0 * speed
        base_lookahead *= 1.0 / (1.0 + 5.0 * path_curvature)
        
        # 误差增益考虑当前模式
        if self.converge_mode:
            error_gain = 1.0 + 0.2 * abs(cross_track_error)
        else:
            error_gain = np.clip(1.0 - 0.05 * abs(cross_track_error), 0.5, 1.2)
        
        self.lookahead_dist = np.clip(
            base_lookahead * error_gain,
            self.min_lookahead,
            self.max_lookahead
        )

    def los_guidance(self, lookahead_point, tangent):
        """改进的LOS引导算法"""
        if lookahead_point is None or tangent is None:
            return self.cur_rot
        
        # 计算横向误差
        cross_track_error = self.compute_cross_track_error(lookahead_point, tangent)
        
        # 检查误差方向变化
        current_sign = 1 if cross_track_error >= 0 else -1
        if current_sign != self.last_heading_error_sign:
            self.pid_wz.reset_integral()
        self.last_heading_error_sign = current_sign
        
        # 计算路径角度和LOS角度
        path_angle = math.atan2(tangent[1], tangent[0])
        
        # 改进的LOS角度计算
        if self.converge_mode:
            los_angle = math.atan2(-cross_track_error, 
                                 max(self.lookahead_dist, 10.0))
        else:
            los_angle = math.atan2(-cross_track_error, self.lookahead_dist)
        
        # 非线性侧滑角补偿
        beta_compensation = math.sin(self.beta) * min(1.0, abs(cross_track_error)/2.0)
        
        desired_heading = path_angle + los_angle - beta_compensation

        '''# 诊断日志输出
        self.get_logger().info(
            f"\n"
            f"横向误差: {cross_track_error:.2f}m (符号:{current_sign})\n"
            f"前视距离: {self.lookahead_dist:.1f}m\n"
            f"路径角度: {math.degrees(path_angle):.1f}° (切线:[{tangent[0]:.2f},{tangent[1]:.2f}])\n"
            f"LOS角度: {math.degrees(los_angle):.1f}° (atan2(-{cross_track_error:.1f}, {self.lookahead_dist:.1f})\n"
            f"侧滑补偿: {math.degrees(beta_compensation):.1f}° (β={math.degrees(self.beta):.1f}°)\n"
            f"最终航向: {math.degrees(desired_heading):.1f}° (当前:{math.degrees(self.cur_rot):.1f}°)\n"
            f"---------------------------------"
        )'''
        
        return self.normalize_angle(desired_heading)

    def publish_static_tf(self):
        """发布 world 到局部坐标系 (-532, 160) 的静态 TF"""
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'world'
        static_transform.child_frame_id = 'local_frame'  # 局部坐标系名称
        static_transform.transform.translation.x = -532.0  # 设置局部坐标系原点
        static_transform.transform.translation.y = 160.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.w = 1.0  # 无旋转
        self.static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info("已发布静态 TF: world → local_frame (-532, 160)")

    def publish_transforms(self):
        """发布TF坐标（小船坐标）"""
        if self.cur_pos is None or self.cur_rot is None:
            return

        # 发布WAMV当前位姿
        t_wamv = TransformStamped()
        t_wamv.header.stamp = self.get_clock().now().to_msg()
        t_wamv.header.frame_id = 'world'
        t_wamv.child_frame_id = 'wamv/base_link'
        t_wamv.transform.translation.x = float(self.cur_pos[0])
        t_wamv.transform.translation.y = float(self.cur_pos[1])
        t_wamv.transform.rotation.z = math.sin(self.cur_rot / 2)
        t_wamv.transform.rotation.w = math.cos(self.cur_rot / 2)
        self.tf_broadcaster.sendTransform(t_wamv)

    def publish_markers(self):
        """发布前视点标记"""
        if self.current_lookahead is not None:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = marker.scale.y = marker.scale.z = 0.5  # 缩小尺寸
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
            marker.color.a = 0.8
            marker.pose.position = Point(
                x=float(self.current_lookahead[0]),
                y=float(self.current_lookahead[1]),
                z=0.0
            )
            self.marker_pub.publish(marker)

    def control_loop(self):
        """带安全监控的平滑控制"""
        # 基本检查
        if self.cur_pos is None or self.cur_rot is None or not self.path_points:
            return

        # 发布TF坐标
        self.publish_transforms()

        # 寻找前视点和最近点
        lookahead_point, tangent = self.find_lookahead_point()
        if lookahead_point is None:
            return
        else:
            self.current_lookahead = lookahead_point

        # 误差计算
        cross_track_error = self.compute_cross_track_error(lookahead_point, tangent)
        distance = np.linalg.norm(lookahead_point - self.cur_pos)
        
        # 自适应前视距离
        self.adaptive_lookahead(cross_track_error, speed=abs(self.last_cmd.linear.x))
        
        # 估计路径曲率
        path_curvature = self.estimate_path_curvature()
        
        # 改进的收敛模式判断
        converge_threshold = self.converge_dist * (1.0 + 0.5 * path_curvature)
        heading_error = self.normalize_angle(math.atan2(tangent[1], tangent[0]) - self.cur_rot)
        
        if self.converge_mode and (abs(cross_track_error) < converge_threshold 
                              and abs(heading_error) < math.radians(15)):
            self.converge_mode = False
            self.get_logger().info("进入正常跟踪模式")
        if abs(cross_track_error) > 5.0:
            self.get_logger().warn("横向误差过大，重置为收敛模式")
            self.converge_mode = True
            self.lookahead_dist = min(20.0, distance*0.5)

        # LOS引导
        desired_heading = self.los_guidance(lookahead_point, tangent)
        heading_error = self.normalize_angle(desired_heading - self.cur_rot)

        # 生成控制指令
        t_now = self.get_clock().now().nanoseconds
        cmd_vel = Twist()
        
        # 速度控制（考虑曲率和误差）
        speed_limit = self.speed_limit * (0.7 if self.converge_mode else 1.0)
        speed_limit *= 1.0 / (1.0 + 8.0 * path_curvature + 1.5 * abs(cross_track_error))  # 高曲率或高误差时减速
        speed_limit = max(speed_limit, 0.5)  # 设置最低速度保证可控性

        forward_speed = np.clip(
            self.pid_vx.control(distance, t_now),
            -speed_limit,
            speed_limit
        )
        
        # 角速度控制
        curvature_gain = 1.0 + 2.0 * path_curvature  # 高曲率时增强转向
        angular_rate = self.pid_wz.control(heading_error * curvature_gain, t_now)
        angular_rate = np.clip(angular_rate, 
                             -self.max_angular_rate, 
                             self.max_angular_rate)
        
        # 侧滑角过大时自动降速
        if abs(self.beta) > math.radians(25):
            forward_speed *= 0.7
            angular_rate *= 0.8

        # 指令平滑
        cmd_vel.linear.x = (self.smooth_factor * forward_speed +
                           (1-self.smooth_factor) * self.last_cmd.linear.x)
        cmd_vel.angular.z = (self.smooth_factor * angular_rate +
                            (1-self.smooth_factor) * self.last_cmd.angular.z)
        
        # 发布指令
        self.cmd_vel_pub.publish(cmd_vel)
        self.last_cmd = cmd_vel

        # 日志输出
        self.get_logger().info(
            f"\n安全控制状态 | 模式: {'收敛' if self.converge_mode else '跟踪'}\n"
            f"误差: 横向={cross_track_error:.2f}m 航向={math.degrees(heading_error):.1f}°\n"
            f"状态: 侧滑角={math.degrees(self.beta):.1f}° 前视距={self.lookahead_dist:.1f}m\n"
            f"曲率: {path_curvature:.3f} 速度={cmd_vel.linear.x:.2f}m/s\n"
            f"控制: 转向={math.degrees(cmd_vel.angular.z):.1f}°/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedPathTrackingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("控制器关闭")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()