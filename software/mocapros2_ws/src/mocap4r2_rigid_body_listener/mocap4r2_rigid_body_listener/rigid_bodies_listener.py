import csv
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from mocap4r2_msgs.msg import RigidBodies

RIGID_BODY_NAME = '5'
CSV_FILE_PATH = os.path.expanduser('~/mocap4r2_ws/rigid_body_data.csv')


class RigidBodiesListener(Node):

    def __init__(self):
        super().__init__('rigid_bodies_listener')

        file_exists = os.path.isfile(CSV_FILE_PATH)
        self.csv_file = open(CSV_FILE_PATH, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if not file_exists:
            self.csv_writer.writerow([
                'stamp_sec', 'stamp_nanosec',
                'pos_x', 'pos_y', 'pos_z',
                'orient_x', 'orient_y', 'orient_z', 'orient_w',
            ])
            self.csv_file.flush()

        self.subscription = self.create_subscription(
            RigidBodies,
            'rigid_bodies',
            self.rigid_bodies_callback,
            qos_profile_sensor_data)

        self.get_logger().info(
            f"Listening on 'rigid_bodies' for rigid body '{RIGID_BODY_NAME}', "
            f'writing to {CSV_FILE_PATH}')
        self.rows_written = 0

    def rigid_bodies_callback(self, msg):
        for rigid_body in msg.rigidbodies:
            if rigid_body.rigid_body_name == RIGID_BODY_NAME:
                pose = rigid_body.pose
                self.csv_writer.writerow([
                    msg.header.stamp.sec, msg.header.stamp.nanosec,
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w,
                ])
                self.csv_file.flush()
                self.rows_written += 1
                self.get_logger().info(
                    f'[{self.rows_written}] pos=({pose.position.x:.3f}, '
                    f'{pose.position.y:.3f}, {pose.position.z:.3f})',
                    throttle_duration_sec=1)
                return
        else:
            self.get_logger().warn(
                f"Rigid body '{RIGID_BODY_NAME}' not found in message", throttle_duration_sec=5)

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RigidBodiesListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
