#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from ahrs.filters import Madgwick
import numpy as np
import csv
from datetime import datetime
from pathlib import Path
import math
import json
import os


class FilterComparison(Node):
    """Compare Madgwick vs Kalman filter for IMU orientation estimation"""

    def __init__(self):
        super().__init__('filter_comparison')

        # Subscribe to raw IMU
        self.subscription = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10)

        # Publishers for both filters
        self.pub_madgwick = self.create_publisher(
            Vector3Stamped, 'gravity_madgwick', 10)
        self.pub_kalman = self.create_publisher(
            Vector3Stamped, 'gravity_kalman', 10)

        # Madgwick filter
        self.madgwick = Madgwick(sampleperiod=1/100)
        self.q_madgwick = np.array([1.0, 0.0, 0.0, 0.0])

        # Kalman filter
        self.q_kalman = np.array([1.0, 0.0, 0.0, 0.0])
        self.P_kalman = np.eye(4) * 0.1  # State covariance
        self.last_time = None

        # Kalman parameters
        self.Q_kalman = np.eye(4) * 0.001  # Process noise (gyro drift)
        self.R_kalman = np.eye(3) * 0.1    # Measurement noise (accel)
        self.P_kalman_prev = self.P_kalman.copy()

        # Convergence detection
        self.kalman_converged = False
        self.convergence_threshold = 1e-6  # When P stops changing
        self.convergence_counter = 0
        self.convergence_frames_needed = 100  # Frames of stable P to consider converged

        # Parameter file
        self.params_file = Path.home() / ".kalman_params.json"
        self.load_kalman_params()

        if self.kalman_converged:
            self.get_logger().info("Loaded converged Kalman parameters")
        else:
            self.get_logger().info("Starting Kalman calibration mode")

        # CSV setup
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = Path.home() / f"filter_comparison_{timestamp}.csv"
        self.csv_file_handle = open(self.csv_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file_handle)
        self.csv_writer.writerow([
            'timestamp',
            'gx', 'gy', 'gz',
            'ax', 'ay', 'az',
            'madgwick_roll_deg', 'madgwick_pitch_deg', 'madgwick_yaw_deg',
            'kalman_roll_deg', 'kalman_pitch_deg', 'kalman_yaw_deg',
            'kalman_converged', 'kalman_P_change', 'convergence_counter'
        ])
        self.csv_file_handle.flush()
        self.get_logger().info(f"Saving filter comparison to: {self.csv_file}")

    def load_kalman_params(self):
        """Load Kalman parameters from JSON if they exist"""
        if self.params_file.exists():
            try:
                with open(self.params_file, 'r') as f:
                    params = json.load(f)
                self.P_kalman = np.array(params['P_kalman'])
                self.Q_kalman = np.array(params['Q_kalman'])
                self.R_kalman = np.array(params['R_kalman'])
                self.kalman_converged = True
                self.get_logger().info(f"Loaded Kalman params from {self.params_file}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load Kalman params: {e}")

    def save_kalman_params(self):
        """Save converged Kalman parameters to JSON"""
        try:
            params = {
                'P_kalman': self.P_kalman.tolist(),
                'Q_kalman': self.Q_kalman.tolist(),
                'R_kalman': self.R_kalman.tolist(),
                'timestamp': datetime.now().isoformat()
            }
            with open(self.params_file, 'w') as f:
                json.dump(params, f, indent=2)
            self.get_logger().info(f"Saved Kalman params to {self.params_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save Kalman params: {e}")

    def check_convergence(self):
        """Check if Kalman filter has converged"""
        if self.kalman_converged:
            return  # Already converged, don't recalculate

        # Monitor change in covariance matrix
        P_change = np.linalg.norm(self.P_kalman - self.P_kalman_prev)

        if P_change < self.convergence_threshold:
            self.convergence_counter += 1
        else:
            self.convergence_counter = 0

        self.P_kalman_prev = self.P_kalman.copy()

        # If stable for enough frames, mark as converged
        if self.convergence_counter >= self.convergence_frames_needed:
            self.kalman_converged = True
            self.save_kalman_params()
            self.get_logger().info("Kalman filter converged!")

    def quat_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        w, x, y, z = q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        sinp = np.clip(sinp, -1.0, 1.0)
        pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def quat_to_rotmat(self, q):
        """Convert quaternion to rotation matrix"""
        w, x, y, z = q
        return np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - z*w),       2*(x*z + y*w)],
            [    2*(x*y + z*w),   1 - 2*(x**2 + z**2),     2*(y*z - x*w)],
            [    2*(x*z - y*w),       2*(y*z + x*w),   1 - 2*(x**2 + y**2)]
        ])

    def project_gravity(self, q):
        """Project gravity to body frame using quaternion"""
        R = self.quat_to_rotmat(q)
        g_world = np.array([0.0, 0.0, -9.81])
        g_body = R.T @ g_world
        return g_body

    def kalman_predict(self, gyro, dt):
        """Kalman filter prediction step with gyro integration"""
        if dt <= 0:
            return

        # Quaternion derivative: q_dot = 0.5 * q * omega_quat
        # omega_quat = [0, wx, wy, wz]
        omega_quat = np.array([0.0, gyro[0], gyro[1], gyro[2]])

        # q_dot = 0.5 * q * omega_quat
        q_dot = 0.5 * self.quat_multiply(self.q_kalman, omega_quat)

        # Predict state: q = q + q_dot * dt
        self.q_kalman = self.q_kalman + q_dot * dt
        self.q_kalman = self.q_kalman / np.linalg.norm(self.q_kalman)

        # Only update covariance if not converged (saves computation)
        if not self.kalman_converged:
            # Simplified Jacobian for EKF
            # F ≈ I + dq/dq_prev * dt (state transition matrix)
            F = np.eye(4)  # Simplified: assume near-identity

            # Predict covariance: P = F * P * F^T + Q
            self.P_kalman = F @ self.P_kalman @ F.T + self.Q_kalman

            # Check for convergence
            self.check_convergence()

    def kalman_update(self, accel):
        """Extended Kalman Filter update step with accelerometer"""
        # Normalize accelerometer
        accel_norm = accel / (np.linalg.norm(accel) + 1e-8)

        # Expected gravity in body frame from current quaternion estimate
        R = self.quat_to_rotmat(self.q_kalman)
        g_world = np.array([0.0, 0.0, -9.81])
        g_expected = R.T @ g_world

        # Measurement function: h(q) = R(q)^T @ g_world
        # We measure accel (which is gravity when stationary)
        z = accel_norm  # Measured acceleration (normalized)
        h = g_expected / (np.linalg.norm(g_expected) + 1e-8)  # Expected measurement

        # Innovation (measurement residual)
        y = z - h  # 3x1 vector

        # Jacobian H: derivative of h with respect to quaternion
        # For simplified EKF, we approximate H as identity for accel measurement
        # In reality, this should be dh/dq which is more complex
        H = np.eye(4)[:3, :]  # 3x4 matrix (extract first 3 rows)

        # Kalman Gain: K = P * H^T / (H * P * H^T + R)
        S = H @ self.P_kalman @ H.T + self.R_kalman  # Innovation covariance
        try:
            K = self.P_kalman @ H.T @ np.linalg.inv(S)  # Kalman gain (4x3)
        except np.linalg.LinAlgError:
            K = np.zeros((4, 3))  # If singular, skip update

        # State update: x = x + K * y
        # For quaternion, we add the correction as small rotation
        correction = K @ y  # 4x1

        # Update quaternion using small rotation approximation
        self.q_kalman = self.q_kalman + correction
        self.q_kalman = self.q_kalman / np.linalg.norm(self.q_kalman)

        # Covariance update: P = (I - K * H) * P
        if not self.kalman_converged:
            I = np.eye(4)
            self.P_kalman = (I - K @ H) @ self.P_kalman

    def quat_multiply(self, q1, q2):
        """Multiply two quaternions: q1 * q2"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    def imu_callback(self, msg):
        """Process raw IMU data with both filters"""

        # Extract raw data
        gyro = np.array([msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z])

        accel = np.array([msg.linear_acceleration.x,
                         msg.linear_acceleration.y,
                         msg.linear_acceleration.z])

        # Calculate dt for Kalman
        current_time = self.get_clock().now()
        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
        else:
            dt = 0.01  # Default 10ms
        self.last_time = current_time

        # Update Madgwick
        self.q_madgwick = self.madgwick.updateIMU(self.q_madgwick, gyr=gyro, acc=accel)

        # Update Kalman: Predict + Update
        self.kalman_predict(gyro, dt)
        self.kalman_update(accel)

        # Convert to Euler angles
        madgwick_roll, madgwick_pitch, madgwick_yaw = self.quat_to_euler(self.q_madgwick)
        kalman_roll, kalman_pitch, kalman_yaw = self.quat_to_euler(self.q_kalman)

        # Convert to degrees
        madgwick_roll_deg = math.degrees(madgwick_roll)
        madgwick_pitch_deg = math.degrees(madgwick_pitch)
        madgwick_yaw_deg = math.degrees(madgwick_yaw)
        kalman_roll_deg = math.degrees(kalman_roll)
        kalman_pitch_deg = math.degrees(kalman_pitch)
        kalman_yaw_deg = math.degrees(kalman_yaw)

        # Project gravity for publishing (still used downstream)
        gravity_madgwick = self.project_gravity(self.q_madgwick)
        gravity_kalman = self.project_gravity(self.q_kalman)

        # Get timestamp
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        # Calculate P change for convergence monitoring
        P_change = np.linalg.norm(self.P_kalman - self.P_kalman_prev) if hasattr(self, 'P_kalman_prev') else 0.0

        # Save to CSV (orientation in degrees + convergence info)
        self.csv_writer.writerow([
            f"{timestamp:.6f}",
            f"{gyro[0]:.6f}", f"{gyro[1]:.6f}", f"{gyro[2]:.6f}",
            f"{accel[0]:.6f}", f"{accel[1]:.6f}", f"{accel[2]:.6f}",
            f"{madgwick_roll_deg:.6f}", f"{madgwick_pitch_deg:.6f}", f"{madgwick_yaw_deg:.6f}",
            f"{kalman_roll_deg:.6f}", f"{kalman_pitch_deg:.6f}", f"{kalman_yaw_deg:.6f}",
            f"{int(self.kalman_converged)}", f"{P_change:.9f}", f"{self.convergence_counter}"
        ])
        self.csv_file_handle.flush()

        # Publish Madgwick result (still gravity for downstream compatibility)
        msg_madgwick = Vector3Stamped()
        msg_madgwick.header = msg.header
        msg_madgwick.header.frame_id = "body"
        msg_madgwick.vector.x = gravity_madgwick[0]
        msg_madgwick.vector.y = gravity_madgwick[1]
        msg_madgwick.vector.z = gravity_madgwick[2]
        self.pub_madgwick.publish(msg_madgwick)

        # Publish Kalman result (still gravity for downstream compatibility)
        msg_kalman = Vector3Stamped()
        msg_kalman.header = msg.header
        msg_kalman.header.frame_id = "body"
        msg_kalman.vector.x = gravity_kalman[0]
        msg_kalman.vector.y = gravity_kalman[1]
        msg_kalman.vector.z = gravity_kalman[2]
        self.pub_kalman.publish(msg_kalman)

    def __del__(self):
        if self.csv_file_handle and not self.csv_file_handle.closed:
            self.csv_file_handle.close()


def main(args=None):
    rclpy.init(args=args)
    comparison = FilterComparison()
    try:
        rclpy.spin(comparison)
    finally:
        if comparison.csv_file_handle and not comparison.csv_file_handle.closed:
            comparison.csv_file_handle.close()
        comparison.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
