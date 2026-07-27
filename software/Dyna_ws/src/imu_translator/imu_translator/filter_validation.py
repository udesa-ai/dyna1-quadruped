#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import numpy as np
import csv
from datetime import datetime
from pathlib import Path
from collections import deque
import math


class FilterValidation(Node):
    """Compare Kalman vs Madgwick against OptiTrack ground truth"""

    def __init__(self):
        super().__init__('filter_validation')

        # Subscribers
        self.sub_mocap = self.create_subscription(
            Vector3Stamped, '/mocap/projected_gravity_body', self.mocap_cb, 10)
        self.sub_kalman = self.create_subscription(
            Vector3Stamped, 'gravity_kalman', self.kalman_cb, 10)
        self.sub_madgwick = self.create_subscription(
            Vector3Stamped, 'gravity_madgwick', self.madgwick_cb, 10)

        # Latest measurements (with timestamps for synchronization)
        self.mocap_data = None
        self.kalman_data = None
        self.madgwick_data = None

        # CSV setup
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = Path.home() / f"filter_validation_{timestamp}.csv"
        self.csv_file_handle = open(self.csv_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file_handle)
        self.csv_writer.writerow([
            'timestamp',
            'mocap_roll_deg', 'mocap_pitch_deg',
            'kalman_roll_deg', 'kalman_pitch_deg',
            'madgwick_roll_deg', 'madgwick_pitch_deg',
            'kalman_roll_error_deg', 'kalman_pitch_error_deg', 'kalman_total_error_deg',
            'madgwick_roll_error_deg', 'madgwick_pitch_error_deg', 'madgwick_total_error_deg'
        ])
        self.csv_file_handle.flush()
        self.get_logger().info(f"Saving validation to: {self.csv_file}")

        # Statistics (running)
        self.window_size = 100  # Last N measurements
        self.kalman_errors = deque(maxlen=self.window_size)
        self.madgwick_errors = deque(maxlen=self.window_size)

    def gravity_to_euler(self, g):
        """
        Convert projected gravity vector to roll and pitch.
        From g_body = R^T @ [0, 0, -9.81], estimate roll and pitch.
        Note: yaw cannot be determined from gravity alone.
        """
        # Normalize
        g_norm = np.linalg.norm(g)
        if g_norm < 1e-6:
            return 0.0, 0.0

        g = g / g_norm
        gx, gy, gz = g

        # Roll and pitch from accelerometer
        roll = math.atan2(gy, gz)
        pitch = math.atan2(-gx, math.sqrt(gy**2 + gz**2))

        return roll, pitch

    def mocap_cb(self, msg):
        """Store OptiTrack measurement (ground truth)"""
        self.mocap_data = {
            'gravity': np.array([msg.vector.x, msg.vector.y, msg.vector.z]),
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        }
        self.check_and_validate()

    def kalman_cb(self, msg):
        """Store Kalman measurement"""
        self.kalman_data = {
            'gravity': np.array([msg.vector.x, msg.vector.y, msg.vector.z]),
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        }
        self.check_and_validate()

    def madgwick_cb(self, msg):
        """Store Madgwick measurement"""
        self.madgwick_data = {
            'gravity': np.array([msg.vector.x, msg.vector.y, msg.vector.z]),
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        }
        self.check_and_validate()

    def check_and_validate(self):
        """When all three filters have data, compute orientation errors"""
        if self.mocap_data is None or self.kalman_data is None or self.madgwick_data is None:
            return

        # Check timestamps are close (within 100ms)
        timestamps = [self.mocap_data['timestamp'],
                     self.kalman_data['timestamp'],
                     self.madgwick_data['timestamp']]
        if max(timestamps) - min(timestamps) > 0.1:
            return  # Data too far apart

        # Extract gravity vectors
        mocap_g = self.mocap_data['gravity']
        kalman_g = self.kalman_data['gravity']
        madgwick_g = self.madgwick_data['gravity']

        # Convert gravity to roll/pitch (orientation)
        mocap_roll, mocap_pitch = self.gravity_to_euler(mocap_g)
        kalman_roll, kalman_pitch = self.gravity_to_euler(kalman_g)
        madgwick_roll, madgwick_pitch = self.gravity_to_euler(madgwick_g)

        # Convert to degrees
        mocap_roll_deg = np.degrees(mocap_roll)
        mocap_pitch_deg = np.degrees(mocap_pitch)
        kalman_roll_deg = np.degrees(kalman_roll)
        kalman_pitch_deg = np.degrees(kalman_pitch)
        madgwick_roll_deg = np.degrees(madgwick_roll)
        madgwick_pitch_deg = np.degrees(madgwick_pitch)

        # Compute orientation errors
        kalman_roll_error = kalman_roll_deg - mocap_roll_deg
        kalman_pitch_error = kalman_pitch_deg - mocap_pitch_deg
        kalman_total_error = np.sqrt(kalman_roll_error**2 + kalman_pitch_error**2)

        madgwick_roll_error = madgwick_roll_deg - mocap_roll_deg
        madgwick_pitch_error = madgwick_pitch_deg - mocap_pitch_deg
        madgwick_total_error = np.sqrt(madgwick_roll_error**2 + madgwick_pitch_error**2)

        # Store for statistics
        self.kalman_errors.append(kalman_total_error)
        self.madgwick_errors.append(madgwick_total_error)

        # Save to CSV
        self.csv_writer.writerow([
            f"{self.mocap_data['timestamp']:.6f}",
            f"{mocap_roll_deg:.6f}", f"{mocap_pitch_deg:.6f}",
            f"{kalman_roll_deg:.6f}", f"{kalman_pitch_deg:.6f}",
            f"{madgwick_roll_deg:.6f}", f"{madgwick_pitch_deg:.6f}",
            f"{kalman_roll_error:.6f}", f"{kalman_pitch_error:.6f}", f"{kalman_total_error:.6f}",
            f"{madgwick_roll_error:.6f}", f"{madgwick_pitch_error:.6f}", f"{madgwick_total_error:.6f}"
        ])
        self.csv_file_handle.flush()

        # Log statistics every 50 measurements
        if len(self.kalman_errors) >= self.window_size:
            kalman_mean = np.mean(self.kalman_errors)
            kalman_std = np.std(self.kalman_errors)
            madgwick_mean = np.mean(self.madgwick_errors)
            madgwick_std = np.std(self.madgwick_errors)

            self.get_logger().info(
                f"Orientation Error (degrees) | "
                f"Kalman: {kalman_mean:.2f}°±{kalman_std:.2f}° | "
                f"Madgwick: {madgwick_mean:.2f}°±{madgwick_std:.2f}° | "
                f"Kalman Better: {kalman_mean < madgwick_mean}"
            )

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
