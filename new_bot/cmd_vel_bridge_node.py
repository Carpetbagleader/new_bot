#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        # This port is for the Pi; you can change when running on Pi
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("CmdVelBridge node started, publishing to Arduino...")

    def cmd_vel_callback(self, msg):
        linear = int(msg.linear.x * 100)
        angular = int(msg.angular.z * 100)
        command = f"L{linear} A{angular}\n"
        self.ser.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
