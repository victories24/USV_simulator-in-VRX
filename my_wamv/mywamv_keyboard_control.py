#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import termios
import tty
import sys
import select

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        
        # 创建发布者
        self.left_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.left_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/left/pos', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.right_pos_pub = self.create_publisher(Float64, '/wamv/thrusters/right/pos', 10)

        # 初始化thrust和pos值
        self.left_thrust = 0.0
        self.left_pos = 0.0
        self.right_thrust = 0.0
        self.right_pos = 0.0
        
        # 设置键盘输入
        self.settings = termios.tcgetattr(sys.stdin)
        
        # 打印提示信息
        self.get_logger().info("Keyboard control node started.")
        self.get_logger().info("Use W/S to control thrust, A/D to control position.")
        self.get_logger().info("Press E to reset, press Q to quit.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'w':  # 增加推力
                    self.left_thrust += 100.0
                    self.right_thrust += 100.0
                elif key == 's':  # 减少推力
                    self.left_thrust -= 100.0
                    self.right_thrust -= 100.0
                elif key == 'a':  # 左转
                    self.left_pos -= 0.1
                    self.right_pos -= 0.1
                elif key == 'd':  # 右转
                    self.left_pos += 0.1
                    self.right_pos += 0.1
                elif key == 'e':  # 重置
                    self.left_thrust = self.left_pos = self.right_thrust = self.right_pos = 0.0    
                elif key == 'q':  # 退出
                    break
                
                # 限制thrust和pos的范围
                self.left_thrust = max(-4000.0, min(4000.0, self.left_thrust))
                self.left_pos = max(-3.14, min(3.14, self.left_pos))
                self.right_thrust = max(-4000.0, min(4000.0, self.right_thrust))
                self.right_pos = max(-3.14, min(3.14, self.right_pos))

                # 发布thrust和pos
                left_thrust_msg = Float64()
                left_thrust_msg.data = self.left_thrust
                self.left_thrust_pub.publish(left_thrust_msg)
                right_thrust_msg = Float64()
                right_thrust_msg.data = self.right_thrust
                self.right_thrust_pub.publish(right_thrust_msg)
                
                left_pos_msg = Float64()
                left_pos_msg.data = self.left_pos
                self.left_pos_pub.publish(left_pos_msg)
                right_pos_msg = Float64()
                right_pos_msg.data = self.right_pos
                self.right_pos_pub.publish(right_pos_msg)

                
                # 打印当前值
                self.get_logger().info(f"Thrust: {self.left_thrust}, Pos: {self.left_pos}")
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            # 退出时重置终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
