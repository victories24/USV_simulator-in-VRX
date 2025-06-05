#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import termios
import tty
import sys
import select

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        
        # 创建发布者，发布Twist消息到/cmd_vel话题
        self.cmd_vel_pub = self.create_publisher(Twist, '/wamv/cmd_vel', 10)

        # 初始化线速度和角速度
        self.linear_x = 0.0  # 前后速度（+前进，-后退）
        self.linear_y = 0.0  # 左右速度（+左移，-右移）
        self.angular_z = 0.0  # 旋转速度（+逆时针，-顺时针）

        # 设置键盘输入
        self.settings = termios.tcgetattr(sys.stdin)
        
        # 打印提示信息
        self.get_logger().info("Keyboard control node started.")
        self.get_logger().info("Use W/S to move forward/backward, A/D to move left/right.")
        self.get_logger().info("Use Q/E to rotate counter-clockwise/clockwise.")
        self.get_logger().info("Press R to reset, press ESC to quit.")

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
                if key == 'w':  # 前进
                    self.linear_x += 0.1
                elif key == 's':  # 后退
                    self.linear_x -= 0.1
                elif key == 'a':  # 左移
                    self.linear_y += 0.1
                elif key == 'd':  # 右移
                    self.linear_y -= 0.1
                elif key == 'q':  # 逆时针旋转
                    self.angular_z += 0.1
                elif key == 'e':  # 顺时针旋转
                    self.angular_z -= 0.1
                elif key == 'r':  # 重置速度
                    self.linear_x = self.linear_y = self.angular_z = 0.0
                elif key == '\x1b':  # ESC键退出
                    break
                
                # 限制速度范围
                self.linear_x = max(-5.0, min(5.0, self.linear_x))
                self.linear_y = max(-5.0, min(5.0, self.linear_y))
                self.angular_z = max(-3.14, min(3.14, self.angular_z))

                # 发布Twist消息
                cmd_vel = Twist()
                cmd_vel.linear.x = self.linear_x
                cmd_vel.linear.y = self.linear_y
                cmd_vel.angular.z = self.angular_z
                self.cmd_vel_pub.publish(cmd_vel)
                
                # 打印当前速度
                self.get_logger().info(
                    f"Linear: x={self.linear_x:.2f}, y={self.linear_y:.2f} | "
                    f"Angular: z={self.angular_z:.2f}"
                )
                
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