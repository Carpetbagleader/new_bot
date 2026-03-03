#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)  # MUST match Arduino BAUDRATE
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("CmdVelBridge node started, publishing to Arduino...")

    def cmd_vel_callback(self, msg):
        # Send your usual motor command
        linear = msg.linear.x
        angular = msg.angular.z
        max_pwm = 255
        left_pwm = int(max(-1.0, min(1.0, linear - angular)) * max_pwm)
        right_pwm = int(max(-1.0, min(1.0, linear + angular)) * max_pwm)
        command = f"m {left_pwm} {right_pwm}\r"
        self.ser.write(command.encode())
        self.get_logger().info(f"Sent: {command.strip()}")

        # READ any encoder lines Arduino prints
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line:  # print non-empty lines
                self.get_logger().info(f"Arduino says: {line}")

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