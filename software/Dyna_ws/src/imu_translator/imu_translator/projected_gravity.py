#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from custom_sensor_msgs.msg import IMUdata
from geometry_msgs.msg import Vector3Stamped
import numpy as np
import csv
from datetime import datetime
from pathlib import Path


class ProjectedGravityPublisher(Node):

    def __init__(self):
        super().__init__('projected_gravity_from_imu')
        self.subscription = self.create_subscription(
            IMUdata, 'IMU', self.imu_callback, 10)
        self.publisher_ = self.create_publisher(
            Vector3Stamped, 'projected_gravity_body', 10)

        self.gravity_magnitude = 9.81

        # CSV setup
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = Path.home() / f"projected_gravity_{timestamp}.csv"
        self.csv_file_handle = None
        self.csv_writer = None
        self._init_csv()
        self.get_logger().info(f"Saving projected gravity to: {self.csv_file}")

    def _init_csv(self):
        self.csv_file_handle = open(self.csv_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file_handle)
        self.csv_writer.writerow(['timestamp', 'gravity_x', 'gravity_y', 'gravity_z'])
        self.csv_file_handle.flush()

    def imu_callback(self, msg):
        roll = msg.roll
        pitch = msg.pitch

        # Matriz de rotación (body to world)
        # R = Rz(0) * Ry(pitch) * Rx(roll)
        # Pero como yaw=0, es solo Ry(pitch) * Rx(roll)

        cos_roll = np.cos(roll)
        sin_roll = np.sin(roll)
        cos_pitch = np.cos(pitch)
        sin_pitch = np.sin(pitch)

        # Matriz de rotación (body to world)
        R = np.array([
            [cos_pitch, sin_roll * sin_pitch, -cos_roll * sin_pitch],
            [0, cos_roll, sin_roll],
            [sin_pitch, -sin_roll * cos_pitch, cos_roll * cos_pitch]
        ])

        # Gravedad en el frame del mundo
        gravity_world = np.array([0.0, 0.0, -self.gravity_magnitude])

        # Proyectar al frame del cuerpo: g_body = R^T * g_world
        gravity_body = R.T @ gravity_world

        # Publicar resultado
        msg_out = Vector3Stamped()
        timestamp = self.get_clock().now().to_msg()
        msg_out.header.stamp = timestamp
        msg_out.header.frame_id = "body"
        msg_out.vector.x = float(gravity_body[0])
        msg_out.vector.y = float(gravity_body[1])
        msg_out.vector.z = float(gravity_body[2])

        self.publisher_.publish(msg_out)

        # Guardar en CSV
        time_sec = timestamp.sec + timestamp.nanosec / 1e9
        self.csv_writer.writerow([
            f"{time_sec:.6f}",
            f"{gravity_body[0]:.6f}",
            f"{gravity_body[1]:.6f}",
            f"{gravity_body[2]:.6f}"
        ])
        self.csv_file_handle.flush()


    def __del__(self):
        if self.csv_file_handle and not self.csv_file_handle.closed:
            self.csv_file_handle.close()


def main(args=None):
    rclpy.init(args=args)
    publisher = ProjectedGravityPublisher()
    try:
        rclpy.spin(publisher)
    finally:
        if publisher.csv_file_handle and not publisher.csv_file_handle.closed:
            publisher.csv_file_handle.close()
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
