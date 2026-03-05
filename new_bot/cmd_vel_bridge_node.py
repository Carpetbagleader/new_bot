#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import serial
import math
import copy


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # ---------------- SERIAL ----------------
        self.ser = serial.Serial('/dev/ttyACM0', 57600, timeout=0.1)

        # ---------------- ROS ----------------
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- LASER FIX ----------------
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.laser_pub = self.create_publisher(LaserScan, '/scan_fixed', 10)

        # ---------------- ROBOT PARAMETERS ----------------
        self.wheel_radius = 0.065             # meters
        self.wheel_base = 0.2794              # meters
        self.encoder_counts_per_rev = 1968.4  # measured

        # ---------------- STATE ----------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

        self.prev_left = None
        self.prev_right = None

        self.last_time = self.get_clock().now()

        # Poll encoders at 20 Hz
        self.create_timer(0.05, self.read_encoders)

        self.get_logger().info("CmdVelBridge READY.")


    # ====================================================
    # CMD_VEL -> MOTOR COMMAND
    # ====================================================
    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        left = linear - (angular * self.wheel_base / 2.0)
        right = linear + (angular * self.wheel_base / 2.0)

        max_input = max(abs(left), abs(right), 1.0)
        left /= max_input
        right /= max_input

        max_pwm = 255
        left_pwm = int(left * max_pwm)
        right_pwm = int(right * max_pwm)

        command = f"m {left_pwm} {right_pwm}\r"
        self.ser.write(command.encode())


    # ====================================================
    # READ ENCODERS
    # ====================================================
    def read_encoders(self):
        try:
            self.ser.write(b'e\r')
            line = self.ser.readline().decode(errors='ignore').strip()
            parts = line.split()
            if len(parts) != 2:
                return

            left_enc = int(parts[0])
            right_enc = int(parts[1])
        except Exception as e:
            self.get_logger().warn(f"Serial read failed: {e}")
            return

        # Always update odom even on first reading
        self.update_odometry(left_enc, right_enc, first_read=(self.prev_left is None))


    # ====================================================
    # ODOMETRY UPDATE
    # ====================================================
    def update_odometry(self, left_enc, right_enc, first_read=False):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # First reading guard
        if self.prev_left is None:
            self.prev_left = left_enc
            self.prev_right = right_enc
            if first_read:
                # Publish initial odom & TF for SLAM
                self.publish_odom_and_tf(0.0, 0.0, 0.0, 0.0)
            return

        # Convert encoder ticks -> radians
        d_left = (left_enc - self.prev_left) * 2.0 * math.pi / self.encoder_counts_per_rev
        d_right = (right_enc - self.prev_right) * 2.0 * math.pi / self.encoder_counts_per_rev

        self.prev_left = left_enc
        self.prev_right = right_enc

        # Update wheel joint angles
        self.left_wheel_angle += d_left
        self.right_wheel_angle += d_right

        # Convert to distance
        d_left_dist = d_left * self.wheel_radius
        d_right_dist = d_right * self.wheel_radius

        d_center = (d_left_dist + d_right_dist) / 2.0
        d_theta = (d_right_dist - d_left_dist) / self.wheel_base

        # Midpoint integration
        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish odom & TF
        if dt > 0:
            self.publish_odom_and_tf(d_center / dt, d_theta / dt, d_center, d_theta)


    # ====================================================
    # PUBLISH ODOM + TF + JOINTS
    # ====================================================
    def publish_odom_and_tf(self, vx, vtheta, d_center=0.0, d_theta=0.0):
        now = self.get_clock().now()

        # ODOM
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vtheta

        # Covariance
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.02
        odom.twist.covariance[0] = 0.1
        odom.twist.covariance[35] = 0.1

        self.odom_pub.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # JOINTS
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_angle, self.right_wheel_angle]
        js.velocity = []
        js.effort = []
        self.joint_pub.publish(js)


    # ====================================================
    # LASER CALLBACK - FIX TIMESTAMPS
    # ====================================================
def laser_callback(self, msg: LaserScan):
    # Use the exact timestamp of the scan
    scan_time = msg.header.stamp

    # Publish a synchronized odom→base_link transform at scan time
    t = TransformStamped()
    t.header.stamp = scan_time
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"

    t.transform.translation.x = self.x
    t.transform.translation.y = self.y
    t.transform.translation.z = 0.0

    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = math.sin(self.theta / 2.0)
    t.transform.rotation.w = math.cos(self.theta / 2.0)

    self.tf_broadcaster.sendTransform(t)

    # Now republish the scan with normal (now) timestamp too
    fixed = copy.deepcopy(msg)
    fixed.header.stamp = self.get_clock().now().to_msg()
    self.laser_pub.publish(fixed)


if __name__ == '__main__':
    main()