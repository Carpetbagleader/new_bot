#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("CmdVelBridge node started, publishing to Arduino...")

        # Timer to periodically ask for encoder values (every 1 second)
        self.create_timer(1.0, self.request_encoders)

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        max_pwm = 255
        left_pwm = int(max(-1.0, min(1.0, linear - angular)) * max_pwm)
        right_pwm = int(max(-1.0, min(1.0, linear + angular)) * max_pwm)
        command = f"m {left_pwm} {right_pwm}\r"
        self.ser.write(command.encode())
        self.get_logger().info(f"Sent: {command.strip()}")

    def request_encoders(self):
        # Send 'e' to Arduino to request encoder values
        self.ser.write(b"e\r")
        # Read all available lines
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                self.get_logger().info(f"Encoder: {line}")

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