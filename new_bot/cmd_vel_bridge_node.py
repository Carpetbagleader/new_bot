#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import serial
import math
import time
import tf_transformations
from rclpy.qos import QoSProfile

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        # Serial to Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)  # MUST match Arduino BAUDRATE
        self.get_logger().info("CmdVelBridge node started, publishing to Arduino...")

        # Subscriber for cmd_vel
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publisher for odometry
        qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)

        # TF broadcaster
        self.tf_pub = self.create_publisher(TransformStamped, '/tf', qos)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Constants
        self.wheel_radius = 0.065       # meters
        self.wheel_base = 0.2794        # meters
        self.ticks_per_rev = 1968       # ticks per wheel revolution
        self.dist_per_tick = 2*math.pi*self.wheel_radius/self.ticks_per_rev

        # Last encoder readings
        self.last_left = 0
        self.last_right = 0

        # Timer to read odom every 0.1s
        self.create_timer(0.1, self.read_odometry)

    def cmd_vel_callback(self, msg):
        # Simple differential drive mixing
        linear = msg.linear.x
        angular = msg.angular.z

        left = linear - angular
        right = linear + angular

        # Scale to PWM range
        max_pwm = 255
        left_pwm = int(max(-1.0, min(1.0, left)) * max_pwm)
        right_pwm = int(max(-1.0, min(1.0, right)) * max_pwm)

        # Send RAW PWM command to ROSArduinoBridge firmware
        command = f"m {left_pwm} {right_pwm}\r"
        self.ser.write(command.encode())
        self.get_logger().info(f"Sent: {command.strip()}")

    def read_odometry(self):
        # Ask Arduino for encoder values
        self.ser.write(b"e\r")  # 'e' command returns: LEFT RIGHT
        line = self.ser.readline().decode('utf-8').strip()
        if line == '':
            return
        try:
            left_ticks, right_ticks = [int(x) for x in line.split()]
        except:
            return

        # Compute delta ticks
        delta_left = left_ticks - self.last_left
        delta_right = right_ticks - self.last_right
        self.last_left = left_ticks
        self.last_right = right_ticks

        # Convert to distances
        d_left = delta_left * self.dist_per_tick
        d_right = delta_right * self.dist_per_tick

        # Odometry calculations
        d_center = (d_left + d_right)/2
        d_theta = (d_right - d_left)/self.wheel_base

        self.x += d_center * math.cos(self.theta + d_theta/2)
        self.y += d_center * math.sin(self.theta + d_theta/2)
        self.theta += d_theta

        # Publish odometry
        now = self.get_clock().now()
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        quat = tf_transformations.quaternion_from_euler(0,0,self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        self.odom_pub.publish(odom)

        # Publish TF
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = quat[0]
        tf.transform.rotation.y = quat[1]
        tf.transform.rotation.z = quat[2]
        tf.transform.rotation.w = quat[3]
        self.tf_pub.publish(tf)

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