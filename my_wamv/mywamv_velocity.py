#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
import numpy as np

class VelocityMonitor(Node):
    def __init__(self):
        super().__init__('velocity_monitor')
        
        # 订阅
        self.imu_sub = self.create_subscription(
            Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, 10)
            
        # 变量初始化
        self.last_gps = None
        self.last_time_sec = None
        self.current_yaw = 0.0
        self.angular_velocity_z = 0.0
        
    def get_yaw_from_imu(self, orientation):
        # 直接从四元数计算偏航角
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        
        # 简化的偏航角计算
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)
        
    def imu_callback(self, msg):
        # 获取当前偏航角
        self.current_yaw = self.get_yaw_from_imu(msg.orientation)
        
        # 获取角速度
        self.angular_velocity_z = msg.angular_velocity.z
        
    def time_to_sec(self, time_msg):
        # 将 Time 消息转换为秒数
        return float(time_msg.sec) + float(time_msg.nanosec) * 1e-9
        
    def gps_callback(self, msg):
        current_time = msg.header.stamp
        current_time_sec = self.time_to_sec(current_time)
        
        if self.last_gps is not None and self.last_time_sec is not None:
            dt = current_time_sec - self.last_time_sec
            
            if dt > 0:
                # 计算大地坐标系下的速度
                lat1, lon1 = np.radians(self.last_gps.latitude), np.radians(self.last_gps.longitude)
                lat2, lon2 = np.radians(msg.latitude), np.radians(msg.longitude)
                
                # 简化的平面近似
                R = 6371000  # 地球半径(米)
                dx = R * (lon2 - lon1) * np.cos((lat1 + lat2)/2)
                dy = R * (lat2 - lat1)
                
                # 大地坐标系下的速度
                v_north = dy / dt
                v_east = dx / dt
                
                # 转换到船体坐标系
                vx = v_east * np.cos(self.current_yaw) + v_north * np.sin(self.current_yaw)
                vy = -v_east * np.sin(self.current_yaw) + v_north * np.cos(self.current_yaw)
                
                # 打印速度信息到终端
                print("\n--- 速度信息 ---")
                print(f"X轴速度(前进方向): {vx:.2f} m/s")
                print(f"Y轴速度(左方向): {vy:.2f} m/s")
                print(f"角速度(偏航): {self.angular_velocity_z:.2f} rad/s")
                print("----------------")
        
        # 更新上一时刻数据
        self.last_gps = msg
        self.last_time_sec = current_time_sec

def main(args=None):
    rclpy.init(args=args)
    node = VelocityMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n监测停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()