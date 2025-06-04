#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class HybridPathPublisher(Node):
    def __init__(self):
        super().__init__('hybrid_path_publisher')
        # 参数定义
        self.area_min = (-550.0, 165.0)
        self.area_max = (-480.0, 230.0)
        self.start_point = (-532.0, 166.0)  # 起点
        self.num_points = 1600              # 增加点数保证平滑
        self.max_curvature = 0.5
        self.has_published = False
        
        # 生成路径
        self.path_points = self.generate_hybrid_path()
        
        # 发布器
        self.publisher = self.create_publisher(Path, '/planning/path', 10)
        self.timer = self.create_timer(0.1, self.publish_path)

    def generate_hybrid_path(self):
        """生成椭圆+交叉+内凹的混合路径"""
        width = self.area_max[0] - self.area_min[0]
        height = self.area_max[1] - self.area_min[1]
        mid_x = (self.area_min[0] + self.area_max[0]) / 2
        mid_y = (self.area_min[1] + self.area_max[1]) / 2

        # 关键点定义
        key_points = [
            # 起点（下部中间偏右）
            self.start_point,
            
            # 下部内凹部分（向右然后凹入）
            (-532.0 + width*0.15, 166.0),
            (-532.0 + width*0.25, 166.0 + height*0.15),
            (-532.0 + width*0.35, 166.0 + height*0.2),  # 凹口底部
            (-532.0 + width*0.45, 166.0 + height*0.15),
            (-532.0 + width*0.55, 166.0),
            
            # 右侧椭圆段（向上）
            (-532.0 + width*0.7, 166.0 + height*0.05),
            (-532.0 + width*0.8, 166.0 + height*0.3),
            (-532.0 + width*0.85, 166.0 + height*0.5),
            (-532.0 + width*0.8, 166.0 + height*0.7),
            
            # 上部的又字型
            (-532.0 + width*0.55, 166.0 + height*0.85),  
            (-532.0 + width*0.35, 166.0 + height*0.75),    # 交叉点
            (-532.0 + width*0.2, 166.0 + height*0.6), 
            (-532.0 + width*0.2, 166.0 + height*0.45),
            (-532.0 + width*0.35, 166.0 + height*0.35),    # 底点
            (-532.0 + width*0.5, 166.0 + height*0.45),
            (-532.0 + width*0.5, 166.0 + height*0.6),  
            (-532.0 + width*0.35, 166.0 + height*0.75), 
            (-532.0 + width*0.15, 166.0 + height*0.85), 
            
            # 左侧椭圆段（向下）
            (-550.0 + width*0.2, 166.0 + height*0.8),
            (-550.0, 166.0 + height*0.5),
            (-550.0 + width*0.05, 166.0 + height*0.1),
            
            # 闭合回起点
            (-532.0, 166.0)
        ]

        # 插值生成路径
        path = []
        n = len(key_points)
        for i in range(n - 1):
            p0 = key_points[max(i-1, 0)]
            p1 = key_points[i]
            p2 = key_points[i+1]
            p3 = key_points[min(i+2, n-1)]
            
            segment_points = self.num_points // (n - 1)
            for j in range(segment_points):
                t = j / segment_points
                point = self.catmull_rom_interp(p0, p1, p2, p3, t)
                path.append(point)
        
        path.append(key_points[0])  # 闭合路径
        return self.smooth_curvature(path)

    def catmull_rom_interp(self, p0, p1, p2, p3, t):
        """Catmull-Rom样条插值"""
        t2 = t * t
        t3 = t2 * t
        return (
            0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t +
            (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2 +
            (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3),
            0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t +
            (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2 +
            (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3)
        )

    def smooth_curvature(self, path, iterations=5):
        """曲率平滑处理"""
        for _ in range(iterations):
            new_path = []
            n = len(path)
            for i in range(n):
                p0 = path[i-2] if i >= 2 else path[n-2+i]
                p1 = path[i-1] if i >= 1 else path[n-1]
                p2 = path[i]
                p3 = path[(i+1)%n]
                
                # 计算当前点曲率
                curvature = self.calculate_curvature(p1, p2, p3)
                if curvature > self.max_curvature:
                    # 插入新的平滑点
                    blend = 0.3
                    new_point = (
                        p2[0]*(1-blend) + (p1[0]+p3[0])/2*blend,
                        p2[1]*(1-blend) + (p1[1]+p3[1])/2*blend
                    )
                    new_path.append(new_point)
                new_path.append(p2)
            path = new_path
        return path

    def calculate_curvature(self, p0, p1, p2):
        """计算曲率"""
        dx1 = p1[0] - p0[0]
        dy1 = p1[1] - p0[1]
        dx2 = p2[0] - p1[0]
        dy2 = p2[1] - p1[1]
        
        cross = dx1 * dy2 - dy1 * dx2
        len1 = math.hypot(dx1, dy1)
        len2 = math.hypot(dx2, dy2)
        
        if len1 * len2 < 1e-6:
            return 0.0
            
        return 2 * abs(cross) / (len1 * len2 * (len1 + len2))

    def calculate_yaw(self, current_point, next_point):
        """计算航向角"""
        dx = next_point[0] - current_point[0]
        dy = next_point[1] - current_point[1]
        return math.atan2(dy, dx)

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'world'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 确保从起点开始
        start_idx = min(range(len(self.path_points)),
                       key=lambda i: math.hypot(
                           self.path_points[i][0]-self.start_point[0],
                           self.path_points[i][1]-self.start_point[1]))
        
        ordered_path = (self.path_points[start_idx:-1] + 
                       self.path_points[:start_idx] + 
                       [self.path_points[start_idx]])
        
        for i, point in enumerate(ordered_path):
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            
            next_idx = (i + 1) % len(ordered_path)
            next_point = ordered_path[next_idx]
            yaw = self.calculate_yaw(point, next_point)
            
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            path_msg.poses.append(pose)
        
        self.publisher.publish(path_msg)
        
        if not self.has_published:
            # 计算最大曲率
            max_curvature = max(
                self.calculate_curvature(
                    ordered_path[i-1] if i > 0 else ordered_path[-2],
                    ordered_path[i],
                    ordered_path[(i+1)%len(ordered_path)]
                )
                for i in range(len(ordered_path))
            )
            
            # 检查首尾连接
            closure_error = math.hypot(ordered_path[0][0]-ordered_path[-1][0],
                                     ordered_path[0][1]-ordered_path[-1][1])
            
            self.get_logger().info(
                f"已发布凹字型+又字型路径:\n"
                f"- 点数: {len(ordered_path)}\n"
                f"- 首尾误差: {closure_error:.6f} 米\n"
                f"- 最大曲率: {max_curvature:.4f} (限制: {self.max_curvature})\n"
                f"- 起点位置: ({ordered_path[0][0]:.2f}, {ordered_path[0][1]:.2f})"
            )
            self.has_published = True

if __name__ == '__main__':
    rclpy.init()
    node = HybridPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Path publisher stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()