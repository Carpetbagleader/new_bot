#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import serial
import math
import time

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)

        # cmd_vel subscriber
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # wheel parameters
        self.wheel_radius = 0.065  # meters
        self.wheel_base = 0.2794  # 11 inches in meters
        self.encoder_counts_per_rev = 1968.4  # 9842/5 rotations

        # odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # store previous encoder values
        self.prev_left = 0
        self.prev_right = 0

        self.get_logger().info("CmdVelBridge node started. Sending commands and publishing odom...")

    def cmd_vel_callback(self, msg):
        # send PWM to Arduino
        linear = msg.linear.x
        angular = msg.angular.z
        left = linear - angular
        right = linear + angular
        max_pwm = 255
        left_pwm = int(max(-1.0, min(1.0, left)) * max_pwm)
        right_pwm = int(max(-1.0, min(1.0, right)) * max_pwm)
        command = f"m {left_pwm} {right_pwm}\r"
        self.ser.write(command.encode())
        self.get_logger().debug(f"Sent: {command.strip()}")

        # read encoders
        self.ser.write(b'e\r')  # request encoder values
        line = self.ser.readline().decode().strip()
        if line.startswith("Encoder:"):
            try:
                parts = line.split()
                left_enc = int(parts[1])
                right_enc = int(parts[2])
                self.update_odometry(left_enc, right_enc)
            except:
                self.get_logger().warn(f"Failed to parse encoder: {line}")

    def update_odometry(self, left_enc, right_enc):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # delta counts
        dl = (left_enc - self.prev_left) * 2 * math.pi * self.wheel_radius / self.encoder_counts_per_rev
        dr = (right_enc - self.prev_right) * 2 * math.pi * self.wheel_radius / self.encoder_counts_per_rev
        self.prev_left = left_enc
        self.prev_right = right_enc

        # differential drive odometry
        dc = (dr + dl) / 2.0
        dtheta = (dr - dl) / self.wheel_base
        self.x += dc * math.cos(self.theta + dtheta / 2.0)
        self.y += dc * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        # publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = dc / dt if dt > 0 else 0.0
        odom.twist.twist.angular.z = dtheta / dt if dt > 0 else 0.0
        self.odom_pub.publish(odom)

        # broadcast TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

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