#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from mocap4r2_msgs.msg import RigidBodies
from joint_msgs.msg import NeuralInput
import numpy as np
import csv
from datetime import datetime
from pathlib import Path

DATA_DIR = Path.cwd() / "src" / "imu_translator" / "data"


class FilterValidation(Node):
    def __init__(self):
        super().__init__('filter_validation')

        # Subscribers
        self.sub_mocap = self.create_subscription(
            RigidBodies, '/rigid_bodies', self.rigid_bodies_cb, 1)
        self.sub_imu = self.create_subscription(
            Imu, 'imu', self.imu_cb, 1)
        self.sub_mocap_vel = self.create_subscription(
            Twist, '/rigid_body_velocity_filter', self.mocap_vel_cb, 1)
        # base_ang_vel actually fed to the neural net, already offset-corrected
        # and axis-remapped by dyna_real_interface.cpp, so this matches what
        # net_interface.py sees instead of recomputing it locally.
        self.sub_net_input = self.create_subscription(
            NeuralInput, 'network_input', self.net_input_cb, 1)

        # Latest measurements (with timestamps for synchronization)
        self.mocap_data = None
        self.imu_data = None
        self.mocap_vel_data = None
        self.net_input_data = None

        # CSV setup
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        self.csv_file = DATA_DIR / f"filter_validation_{timestamp}.csv"
        self.csv_file_handle = open(self.csv_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file_handle)
        self.csv_writer.writerow([
            'timestamp',
            'gx', 'gy', 'gz',
            'ax', 'ay', 'az',
            'imu_wx', 'imu_wy', 'imu_wz',
            'mocap_wx', 'mocap_wy', 'mocap_wz',
            'mocap_vx', 'mocap_vy', 'mocap_vz',
            'mocap_px', 'mocap_py', 'mocap_pz',
            'mocap_qw', 'mocap_qx', 'mocap_qy', 'mocap_qz',
        ])
        self.csv_file_handle.flush()
        self.get_logger().info(f"Saving validation to: {self.csv_file}")

    def rigid_bodies_cb(self, msg):
        """Store OptiTrack pose (ground truth): orientation and position."""
        if not msg.rigidbodies:
            return
        pose = msg.rigidbodies[0].pose
        q = pose.orientation
        p = pose.position
        self.mocap_data = {
            'quaternion': np.array([q.w, q.x, q.y, q.z]),
            'position': np.array([p.x, p.y, p.z]),
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        }
        self.check_and_validate()

    def imu_cb(self, msg):
        """Store raw IMU measurement (sensor frame, uncorrected)."""
        self.imu_data = {
            'gyro': np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]),
            'accel': np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]),
            # uart_bridge_node.cpp never sets header.stamp on this topic (always 0),
            # so use reception time instead, same as mocap_vel_cb.
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        self.check_and_validate()

    def net_input_cb(self, msg):
        """Store base_ang_vel exactly as published to the neural net by
        dyna_real_interface.cpp. NeuralInput has no header, so use reception
        time as its timestamp."""
        self.net_input_data = {
            'base_ang_vel': np.array([msg.base_ang_vel_x, msg.base_ang_vel_y, msg.base_ang_vel_z]),
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        self.check_and_validate()

    def mocap_vel_cb(self, msg):
        """Store OptiTrack-derived body-frame angular velocity (ground truth).
        Twist has no header, so use reception time as its timestamp."""
        self.mocap_vel_data = {
            'ang_vel': np.array([msg.angular.x, msg.angular.y, msg.angular.z]),
            'lin_vel': np.array([msg.linear.x, msg.linear.y, msg.linear.z]),
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        self.check_and_validate()

    def check_and_validate(self):

        if (self.mocap_data is None or self.imu_data is None
                or self.mocap_vel_data is None or self.net_input_data is None):
            return

        # Check timestamps are close (within 100ms)
        timestamps = [self.mocap_data['timestamp'],
                     self.imu_data['timestamp'],
                     self.mocap_vel_data['timestamp'],
                     self.net_input_data['timestamp']]
        if max(timestamps) - min(timestamps) > 0.1:
            return  # Data too far apart

        # Extract orientation quaternion (w, x, y, z) and position
        mocap_q = self.mocap_data['quaternion']
        mocap_p = self.mocap_data['position']

        # Save to CSV
        gyro = self.imu_data['gyro']
        accel = self.imu_data['accel']
        imu_w = self.net_input_data['base_ang_vel']
        mocap_w = self.mocap_vel_data['ang_vel']
        mocap_v = self.mocap_vel_data['lin_vel']
        self.csv_writer.writerow([
            f"{self.mocap_data['timestamp']:.6f}",
            f"{gyro[0]:.6f}", f"{gyro[1]:.6f}", f"{gyro[2]:.6f}",
            f"{accel[0]:.6f}", f"{accel[1]:.6f}", f"{accel[2]:.6f}",
            f"{imu_w[0]:.6f}", f"{imu_w[1]:.6f}", f"{imu_w[2]:.6f}",
            f"{mocap_w[0]:.6f}", f"{mocap_w[1]:.6f}", f"{mocap_w[2]:.6f}",
            f"{mocap_v[0]:.6f}", f"{mocap_v[1]:.6f}", f"{mocap_v[2]:.6f}",
            f"{mocap_p[0]:.6f}", f"{mocap_p[1]:.6f}", f"{mocap_p[2]:.6f}",
            f"{mocap_q[0]:.6f}", f"{mocap_q[1]:.6f}", f"{mocap_q[2]:.6f}", f"{mocap_q[3]:.6f}",
        ])
        self.csv_file_handle.flush()


    def __del__(self):
        if self.csv_file_handle and not self.csv_file_handle.closed:
            self.csv_file_handle.close()


def main(args=None):
    rclpy.init(args=args)
    validation = FilterValidation()
    try:
        rclpy.spin(validation)
    finally:
        if validation.csv_file_handle and not validation.csv_file_handle.closed:
            validation.csv_file_handle.close()
        validation.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
