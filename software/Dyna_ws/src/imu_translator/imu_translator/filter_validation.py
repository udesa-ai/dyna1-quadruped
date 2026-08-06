#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped, Twist
from sensor_msgs.msg import Imu
import numpy as np
import csv
from datetime import datetime
from pathlib import Path

DATA_DIR = Path.cwd() / "src" / "imu_translator" / "data"


class FilterValidation(Node):
    """Compare Kalman vs Madgwick against OptiTrack ground truth"""

    def __init__(self):
        super().__init__('filter_validation')

        # Subscribers
        self.sub_mocap = self.create_subscription(
            QuaternionStamped, '/mocap/orientation', self.mocap_cb, 10)
        self.sub_kalman = self.create_subscription(
            QuaternionStamped, 'orientation_kalman', self.kalman_cb, 10)
        self.sub_madgwick = self.create_subscription(
            QuaternionStamped, 'orientation_madgwick', self.madgwick_cb, 10)
        self.sub_imu = self.create_subscription(
            Imu, 'imu', self.imu_cb, 10)
        self.sub_mocap_vel = self.create_subscription(
            Twist, '/rigid_body_velocity', self.mocap_vel_cb, 10)

        # Gyro calibration offsets (deg/s, raw sensor frame), same as imu_cb()
        # in dyna_real_interface.cpp, so base_ang_vel here matches what would
        # actually reach the neural net without needing that node running.
        self.declare_parameter("gyro_offset_x", 0.0)
        self.declare_parameter("gyro_offset_y", 0.0)
        self.declare_parameter("gyro_offset_z", 0.0)
        self.gyro_offset = np.array([
            self.get_parameter("gyro_offset_x").value,
            self.get_parameter("gyro_offset_y").value,
            self.get_parameter("gyro_offset_z").value,
        ])

        # Latest measurements (with timestamps for synchronization)
        self.mocap_data = None
        self.kalman_data = None
        self.madgwick_data = None
        self.imu_data = None
        self.mocap_vel_data = None

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
            'mocap_qw', 'mocap_qx', 'mocap_qy', 'mocap_qz',
            'kalman_qw', 'kalman_qx', 'kalman_qy', 'kalman_qz',
            'madgwick_qw', 'madgwick_qx', 'madgwick_qy', 'madgwick_qz',
        ])
        self.csv_file_handle.flush()
        self.get_logger().info(f"Saving validation to: {self.csv_file}")

    def mocap_cb(self, msg):
        """Store OptiTrack measurement (ground truth)"""
        q = msg.quaternion
        self.mocap_data = {
            'quaternion': np.array([q.w, q.x, q.y, q.z]),
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        }
        self.check_and_validate()

    def kalman_cb(self, msg):
        """Store Kalman measurement"""
        q = msg.quaternion
        self.kalman_data = {
            'quaternion': np.array([q.w, q.x, q.y, q.z]),
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        }
        self.check_and_validate()

    def madgwick_cb(self, msg):
        """Store Madgwick measurement"""
        q = msg.quaternion
        self.madgwick_data = {
            'quaternion': np.array([q.w, q.x, q.y, q.z]),
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        }
        self.check_and_validate()

    def imu_cb(self, msg):
        """Store raw IMU measurement, plus the body-frame angular velocity
        equivalent to base_ang_vel (same axis remap + deg->rad conversion
        + offset subtraction as imu_cb() in dyna_real_interface.cpp),
        computed locally so it does not depend on the neural net running."""
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        gyro_corrected = gyro - self.gyro_offset

        base_ang_vel = np.array([
            (gyro_corrected[2] / 180.0) * np.pi,
            (gyro_corrected[0] / 180.0) * np.pi,
            (gyro_corrected[1] / 180.0) * np.pi,
        ])

        self.imu_data = {
            'gyro': gyro,
            'accel': np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]),
            'base_ang_vel': base_ang_vel,
            # uart_bridge_node.cpp never sets header.stamp on this topic (always 0),
            # so use reception time instead, same as mocap_vel_cb.
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        self.check_and_validate()

    def mocap_vel_cb(self, msg):
        """Store OptiTrack-derived body-frame angular velocity (ground truth).
        Twist has no header, so use reception time as its timestamp."""
        self.mocap_vel_data = {
            'ang_vel': np.array([msg.angular.x, msg.angular.y, msg.angular.z]),
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        self.check_and_validate()

    def check_and_validate(self):
        """When all three filters have data, compute orientation errors"""
        if (self.mocap_data is None or self.kalman_data is None
                or self.madgwick_data is None or self.imu_data is None
                or self.mocap_vel_data is None):
            return

        # Check timestamps are close (within 100ms)
        timestamps = [self.mocap_data['timestamp'],
                     self.kalman_data['timestamp'],
                     self.madgwick_data['timestamp'],
                     self.imu_data['timestamp'],
                     self.mocap_vel_data['timestamp']]
        if max(timestamps) - min(timestamps) > 1.0:
            return  # Data too far apart

        # Extract orientation quaternions (w, x, y, z)
        mocap_q = self.mocap_data['quaternion']
        kalman_q = self.kalman_data['quaternion']
        madgwick_q = self.madgwick_data['quaternion']

        # Save to CSV
        gyro = self.imu_data['gyro']
        accel = self.imu_data['accel']
        imu_w = self.imu_data['base_ang_vel']
        mocap_w = self.mocap_vel_data['ang_vel']
        self.csv_writer.writerow([
            f"{self.mocap_data['timestamp']:.6f}",
            f"{gyro[0]:.6f}", f"{gyro[1]:.6f}", f"{gyro[2]:.6f}",
            f"{accel[0]:.6f}", f"{accel[1]:.6f}", f"{accel[2]:.6f}",
            f"{imu_w[0]:.6f}", f"{imu_w[1]:.6f}", f"{imu_w[2]:.6f}",
            f"{mocap_w[0]:.6f}", f"{mocap_w[1]:.6f}", f"{mocap_w[2]:.6f}",
            f"{mocap_q[0]:.6f}", f"{mocap_q[1]:.6f}", f"{mocap_q[2]:.6f}", f"{mocap_q[3]:.6f}",
            f"{kalman_q[0]:.6f}", f"{kalman_q[1]:.6f}", f"{kalman_q[2]:.6f}", f"{kalman_q[3]:.6f}",
            f"{madgwick_q[0]:.6f}", f"{madgwick_q[1]:.6f}", f"{madgwick_q[2]:.6f}", f"{madgwick_q[3]:.6f}",
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
