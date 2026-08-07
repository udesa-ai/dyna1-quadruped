import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3Stamped
import csv
from datetime import datetime
from pathlib import Path

DATA_DIR =  Path.cwd() / "src" / "mocap4r2_vel_grav_listener" / "data"


class VelGravListener(Node):
    """Log mocap-derived body velocity (/rigid_body_velocity, published by
    either opti_vel's velocity_publisher or velocity's velocity_calc - same
    topic/type, whichever is running), its filtered counterpart
    (/rigid_body_velocity_filter), and projected_gravity_publisher's
    projected gravity, to a single CSV."""

    def __init__(self):
        super().__init__('vel_grav_listener')

        self.sub_vel = self.create_subscription(
            Twist, '/rigid_body_velocity', self.vel_cb, 10)
        self.sub_vel_fil = self.create_subscription(
                    Twist, '/rigid_body_velocity_filter', self.vel_fil_cb, 10)
        self.sub_grav = self.create_subscription(
            Vector3Stamped, '/mocap/projected_gravity_body', self.grav_cb, 10)

        self.vel_data = None
        self.vel_fil_data = None
        self.grav_data = None

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        self.csv_file = DATA_DIR / f"vel_grav_{timestamp}.csv"
        self.csv_file_handle = open(self.csv_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file_handle)
        self.csv_writer.writerow([
            'timestamp',
            'lin_vel_x', 'lin_vel_y', 'lin_vel_z',
            'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
            'lin_vel_fil_x', 'lin_vel_fil_y', 'lin_vel_fil_z',
            'ang_vel_fil_x', 'ang_vel_fil_y', 'ang_vel_fil_z',
            'gravity_x', 'gravity_y', 'gravity_z',
        ])
        self.csv_file_handle.flush()
        self.get_logger().info(f"Saving velocity + projected gravity to: {self.csv_file}")

        self.rows_written = 0

    def vel_cb(self, msg):
        """Twist has no header, so its own reception doesn't drive a write -
        just cache it for whenever grav_cb fires next."""
        self.get_logger().info(
            "Received /rigid_body_velocity", throttle_duration_sec=1)
        self.vel_data = {
            'linear': (msg.linear.x, msg.linear.y, msg.linear.z),
            'angular': (msg.angular.x, msg.angular.y, msg.angular.z),
        }

    def vel_fil_cb(self, msg):
        """Same idea as vel_cb, but for the filtered velocity topic."""
        self.get_logger().info(
            "Received /rigid_body_velocity_filter", throttle_duration_sec=1)
        self.vel_fil_data = {
            'linear': (msg.linear.x, msg.linear.y, msg.linear.z),
            'angular': (msg.angular.x, msg.angular.y, msg.angular.z),
        }

    def grav_cb(self, msg):
        self.get_logger().info(
            "Received /mocap/projected_gravity_body", throttle_duration_sec=1)
        self.grav_data = {
            'gravity': (msg.vector.x, msg.vector.y, msg.vector.z),
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        }
        self.write_row()

    def write_row(self):
        if self.vel_data is None or self.vel_fil_data is None or self.grav_data is None:
            return

        lin = self.vel_data['linear']
        ang = self.vel_data['angular']
        lin_fil = self.vel_fil_data['linear']
        ang_fil = self.vel_fil_data['angular']
        grav = self.grav_data['gravity']

        self.csv_writer.writerow([
            f"{self.grav_data['timestamp']:.6f}",
            f"{lin[0]:.6f}", f"{lin[1]:.6f}", f"{lin[2]:.6f}",
            f"{ang[0]:.6f}", f"{ang[1]:.6f}", f"{ang[2]:.6f}",
            f"{lin_fil[0]:.6f}", f"{lin_fil[1]:.6f}", f"{lin_fil[2]:.6f}",
            f"{ang_fil[0]:.6f}", f"{ang_fil[1]:.6f}", f"{ang_fil[2]:.6f}",
            f"{grav[0]:.6f}", f"{grav[1]:.6f}", f"{grav[2]:.6f}",
        ])
        self.csv_file_handle.flush()
        self.rows_written += 1
        self.get_logger().info(
            f"[{self.rows_written}] "
            f"lin_vel=({lin[0]:.3f}, {lin[1]:.3f}, {lin[2]:.3f}) "
            f"ang_vel=({ang[0]:.3f}, {ang[1]:.3f}, {ang[2]:.3f}) "
            f"lin_vel_fil=({lin_fil[0]:.3f}, {lin_fil[1]:.3f}, {lin_fil[2]:.3f}) "
            f"ang_vel_fil=({ang_fil[0]:.3f}, {ang_fil[1]:.3f}, {ang_fil[2]:.3f}) "
            f"gravity=({grav[0]:.3f}, {grav[1]:.3f}, {grav[2]:.3f})",
            throttle_duration_sec=1)

    def destroy_node(self):
        if self.csv_file_handle and not self.csv_file_handle.closed:
            self.csv_file_handle.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VelGravListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
