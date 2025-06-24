#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Header

class FigureEightPublisher(Node):
    def __init__(self):
        super().__init__('figure_eight_publisher')
        self.center = (-532.0, 200.0)  # 局部坐标系中心点 (ENU坐标，单位：米)
        self.radius = 25.0             # 8字形半径 (米)
        self.aspect_ratio = 1.5        # 长宽比
        self.num_points = 200          # 增加点数使方向更平滑
        
        # 发布Path消息
        self.publisher = self.create_publisher(
            Path, 
            '/planning/path', 
            10
        )
        self.timer = self.create_timer(0.1, self.publish_path)

    def calculate_tangent(self, t):
        """精确计算8字形路径在参数t处的切线方向"""
        # 8字形参数方程：
        # x = radius * sin(t)*cos(t) = radius * 0.5*sin(2t)
        # y = radius * aspect_ratio * sin(t)
        
        # 导数计算（切线向量）
        dx_dt = self.radius * math.cos(2*t)  # d(sin(t)cos(t))/dt = cos(2t)
        dy_dt = self.radius * self.aspect_ratio * math.cos(t)
        
        # 返回归一化的切线角度
        return math.atan2(dy_dt, dx_dt)

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'world'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(self.num_points):
            t = 2 * math.pi * i / self.num_points
            
            # 计算位置
            x = self.center[0] + self.radius * math.sin(t) * math.cos(t)
            y = self.center[1] + self.radius * self.aspect_ratio * math.sin(t)
            
            # 计算精确的切线方向
            yaw = self.calculate_tangent(t)
            
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            path_msg.poses.append(pose)
        
        self.publisher.publish(path_msg)
        self.log_path_info(path_msg)

    def log_path_info(self, path_msg):
        """记录路径关键信息"""
        if not path_msg.poses:
            return
            
        # 检查起点和中间点的方向一致性
        check_points = [0, len(path_msg.poses)//4, len(path_msg.poses)//2]
        for i in check_points:
            pose = path_msg.poses[i]
            yaw = 2 * math.atan2(pose.pose.orientation.z, 
                                 pose.pose.orientation.w)
            next_pose = path_msg.poses[(i+1)%len(path_msg.poses)]
            actual_dir = math.atan2(
                next_pose.pose.position.y - pose.pose.position.y,
                next_pose.pose.position.x - pose.pose.position.x
            )
            deviation = math.degrees(abs(self.normalize_angle(yaw - actual_dir)))
            
            self.get_logger().info(
                f"点{i}: 理论方向={math.degrees(yaw):.1f}° "
                f"实际方向={math.degrees(actual_dir):.1f}° "
                f"偏差={deviation:.1f}°"
            )

    def normalize_angle(self, angle):
        """角度归一化到[-π, π]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    node = FigureEightPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()