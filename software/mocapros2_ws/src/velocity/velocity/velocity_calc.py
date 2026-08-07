#!/usr/bin/env python3
"""
optitrack_velocity_node.py
──────────────────────────
ROS2 Humble node that subscribes to /rigid_bodies (mocap4r2_msgs/RigidBodies)
and publishes:
  - /velocity/raw          (TwistStamped) – naive finite-difference in robot frame
  - /rigid_body_velocity   (Twist)        – per-axis Kalman filter in robot frame
                                            (same topic + type as opti_vel's
                                            velocity_publisher, drop-in replacement)

Linear velocity:  v_body  = R(q)ᵀ · (Δpos / Δt), Kalman-filtered per axis.
Angular velocity: q_Δ = q_prev⁻¹ ⊗ q_curr → rotation vector → /Δt → ω_body,
                  then a first-order low-pass filter per axis.

Both are expressed in the rigid-body (robot) frame.

Parameters (set via ROS2 params or command-line --ros-args -p key:=value):
  rigid_body_name  (str,   default "ground")   name to match in /rigid_bodies
  kf_pos_noise     (float, default 1e-4)   process noise for position states
  kf_vel_noise     (float, default 1e-2)   process noise for velocity states
  kf_meas_noise    (float, default 1e-6)   measurement noise (OptiTrack ~0.1 mm)
  ang_lpf_cutoff_hz(float, default 5.0)    angular velocity low-pass cutoff (Hz)
  max_dt           (float, default 0.5)    skip update if gap > this (seconds)

Usage:
  ros2 run velocity velocity_calc \
      --ros-args -p rigid_body_name:=MyRobot
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from mocap4r2_msgs.msg import RigidBodies


# ─────────────────────────────────────────────────────────────────
#  Quaternion helpers  (scipy convention: [x, y, z, w])
# ─────────────────────────────────────────────────────────────────

def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """Return 3×3 rotation matrix from quaternion [x,y,z,w]."""
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate of quaternion [x,y,z,w]."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of two quaternions [x,y,z,w]."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    ])


def quat_to_rotvec(q: np.ndarray) -> np.ndarray:
    """
    Convert quaternion [x,y,z,w] to its rotation vector (axis * angle).
    angle = 2·atan2(|v|, w) always comes out in [0, π] since |v| ≥ 0, so
    this is automatically the shortest-path rotation - no branch/wrap
    discontinuity like Euler angles have at ±π.
    """
    v = q[:3]
    w = q[3]
    vnorm = np.linalg.norm(v)
    if vnorm < 1e-12:
        return 2.0 * v          # small-angle: angle ≈ 2·vnorm, axis ≈ v/vnorm
    angle = 2.0 * np.arctan2(vnorm, w)
    return (angle / vnorm) * v


def angular_velocity_body(q_prev: np.ndarray,
                           q_curr: np.ndarray,
                           dt: float) -> np.ndarray:
    """
    Angular velocity in body frame via the relative-rotation method:
      q_Δ    = q_prev⁻¹ ⊗ q_curr   (rotation from k-1 to k, expressed in
                                     the q_prev/body frame)
      ω_body = rotvec(q_Δ) / dt
    Exact for a constant-rate rotation over [k-1, k], unlike differentiating
    q directly (q̇ ≈ Δq/dt), which is only a first-order approximation and
    is prone to cancellation error when q_curr ≈ q_prev.
    """
    q_delta = quat_multiply(quat_conjugate(q_prev), q_curr)
    return quat_to_rotvec(q_delta) / dt


class LowPassFilter:
    """First-order (RC) low-pass filter, one per axis.
    cutoff_hz has direct physical meaning: signal content changing faster
    than cutoff_hz is attenuated, content slower passes through - so it's
    set from how fast real body rotations of interest are, not from an
    abstract noise/process-noise ratio like the Kalman filter needed.
    """
    def __init__(self, cutoff_hz: float):
        self.rc = 1.0 / (2.0 * np.pi * cutoff_hz)
        self.y = None

    def reset(self):
        """Drop the held state so the next sample is taken as-is instead
        of being blended with a value from before a tracking gap."""
        self.y = None

    def filter(self, x: float, dt: float) -> float:
        if self.y is None:
            self.y = x
            return self.y
        alpha = dt / (self.rc + dt)
        self.y = self.y + alpha * (x - self.y)
        return self.y


# ─────────────────────────────────────────────────────────────────
#  Per-axis constant-velocity Kalman filter (used for linear velocity)
#  State:       [position, velocity]
#  Measurement: [position]
# ─────────────────────────────────────────────────────────────────

class AxisKF:
    def __init__(self, pos_noise: float, vel_noise: float, meas_noise: float):
        self.Q = np.diag([pos_noise, vel_noise])   # process noise
        self.R = np.array([[meas_noise]])           # measurement noise
        self.P = np.eye(2) * 1.0                   # initial covariance (generous)
        self.x = np.zeros(2)                       # [pos, vel]
        self._initialized = False

    def initialize(self, position: float):
        self.x = np.array([position, 0.0])
        self._initialized = True

    def predict_and_update(self, measured_pos: float, dt: float):
        if not self._initialized:
            self.initialize(measured_pos)
            return measured_pos, 0.0

        # ── Predict ──────────────────────────────────────────────
        F = np.array([[1.0, dt],
                      [0.0, 1.0]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

        # ── Update ───────────────────────────────────────────────
        H = np.array([[1.0, 0.0]])
        y = measured_pos - (H @ self.x)[0]          # innovation
        S = (H @ self.P @ H.T) + self.R
        K = (self.P @ H.T) / S[0, 0]               # Kalman gain (2×1)
        self.x = self.x + K.flatten() * y
        self.P = (np.eye(2) - np.outer(K.flatten(), H)) @ self.P

        return self.x[0], self.x[1]                 # filtered pos, vel


# ─────────────────────────────────────────────────────────────────
#  ROS2 Node
# ─────────────────────────────────────────────────────────────────
class OptiTrackVelocityNode(Node):

    def __init__(self):
        super().__init__("optitrack_velocity_node")

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter("rigid_body_name", '5')
        self.declare_parameter("kf_pos_noise",  1e-4)
        self.declare_parameter("kf_vel_noise",  1e-2)
        self.declare_parameter("kf_meas_noise", 1e-6)
        self.declare_parameter("ang_lpf_cutoff_hz", 10.0)
        self.declare_parameter("max_dt",        0.5)

        self.target_name = self.get_parameter("rigid_body_name").value
        pos_noise     = self.get_parameter("kf_pos_noise").value
        vel_noise     = self.get_parameter("kf_vel_noise").value
        meas_noise    = self.get_parameter("kf_meas_noise").value
        ang_cutoff_hz = self.get_parameter("ang_lpf_cutoff_hz").value
        self.max_dt   = self.get_parameter("max_dt").value

        # ── Kalman filters: one per translational axis (x, y, z) ─
        self.kf = [AxisKF(pos_noise, vel_noise, meas_noise) for _ in range(3)]

        # ── Low-pass filters for angular velocity (body frame axes) ─
        # ang_lpf_cutoff_hz has direct physical meaning (Hz) instead of an
        # abstract noise ratio, and - unlike the Kalman filter previously
        # used here - has no internal covariance state that can grow
        # unbounded and let a spike back through after a run of rejections.
        self.lpf_ang = [LowPassFilter(ang_cutoff_hz) for _ in range(3)]

        # ── State memory ─────────────────────────────────────────
        self.prev_pos  = None   # np.ndarray [3]
        self.prev_quat = None   # np.ndarray [x,y,z,w]
        self.prev_time = None   # float (seconds)

        # ── Publishers ───────────────────────────────────────────
        qos = rclpy.qos.QoSProfile(depth=10)
        self.pub_raw = self.create_publisher(TwistStamped,
                                             "velocity/raw", qos)
        self.pub_filt = self.create_publisher(Twist,
                                              "/rigid_body_velocity_filter", qos)

        # ── Subscriber ───────────────────────────────────────────
        self.sub = self.create_subscription(RigidBodies, 'rigid_bodies',
                                            self._pose_cb,
                                            qos)

        self.get_logger().info(
            f"Listening on rigid_bodies for '{self.target_name}'\n"
            f"  KF noise – pos: {pos_noise}, vel: {vel_noise}, "
            f"meas: {meas_noise}\n"
            f"  angular LPF cutoff: {ang_cutoff_hz} Hz"
        )

    # ──────────────────────────────────────────────────────────────
    def _pose_cb(self, msg: RigidBodies):
        target_rb = next((rb for rb in msg.rigidbodies if rb.rigid_body_name == self.target_name), None)
        if not target_rb:
            self.get_logger().warn(
                f"Target rigid body '{self.target_name}' not found in message",
                throttle_duration_sec=5.0)
            return
        
        # ── Extract current pose ──────────────────────────────────
        p = target_rb.pose.position
        o = target_rb.pose.orientation
        curr_pos  = np.array([p.x, p.y, p.z])
        curr_quat = np.array([o.x, o.y, o.z, o.w])  # [x,y,z,w]

        # Normalise quaternion (OptiTrack is already normalised, but be safe)
        curr_quat /= np.linalg.norm(curr_quat)

        # Quaternions have double cover (q and -q are the same rotation).
        # OptiTrack can flip sign between consecutive frames, which turns
        # (q_curr - q_prev) into ~2*q instead of a small delta - spiking
        # the finite-difference angular velocity to ~2/dt (~100 rad/s at
        # OptiTrack framerates) even while stationary. Force continuity
        # with the previous quaternion before differentiating.
        if self.prev_quat is not None and np.dot(curr_quat, self.prev_quat) < 0.0:
            curr_quat = -curr_quat

        stamp  = msg.header.stamp
        t_curr = stamp.sec + stamp.nanosec * 1e-9

        # ── First message: just store and return ─────────────────
        if self.prev_pos is None:
            self.prev_pos  = curr_pos
            self.prev_quat = curr_quat
            self.prev_time = t_curr
            # Seed the position KFs with the first measurement
            for i, kf in enumerate(self.kf):
                kf.initialize(curr_pos[i])
            return

        dt = t_curr - self.prev_time
        if dt <= 0.0 or dt > self.max_dt:
            # Stale or duplicate message – reset and wait
            self.get_logger().warn(
                f"Skipping pose: dt={dt:.4f}s (max_dt={self.max_dt}s)",
                throttle_duration_sec=2.0)
            self.prev_pos  = curr_pos
            self.prev_quat = curr_quat
            self.prev_time = t_curr

            for lpf in self.lpf_ang:
                lpf.reset()

            return

        # ─────────────────────────────────────────────────────────
        #  Rotation matrix for this timestep (global → body)
        #  R maps a global vector to the body frame via R.T @ v
        # ─────────────────────────────────────────────────────────
        R = quat_to_rotmat(curr_quat)   # body ← global is R.T

        # ─────────────────────────────────────────────────────────
        #  RAW velocity (naive finite difference, body frame)
        # ─────────────────────────────────────────────────────────
        delta_pos_global = curr_pos - self.prev_pos
        lin_vel_global   = delta_pos_global / dt
        lin_vel_body_raw = R.T @ lin_vel_global        # → robot frame

        ang_vel_body_raw = angular_velocity_body(
            self.prev_quat, curr_quat, dt)              # already body frame

        # ─────────────────────────────────────────────────────────
        #  FILTERED velocity (Kalman, body frame)
        #
        #  Strategy: run KF on global positions, then rotate the
        #  filtered velocities into body frame at publish time.
        #  This keeps each KF axis independent and well-conditioned.
        # ─────────────────────────────────────────────────────────
        filtered_pos_g = np.zeros(3)
        filtered_vel_g = np.zeros(3)
        for i, kf in enumerate(self.kf):
            fp, fv = kf.predict_and_update(curr_pos[i], dt)
            filtered_pos_g[i] = fp
            filtered_vel_g[i] = fv

        lin_vel_body_filt = R.T @ filtered_vel_g       # → robot frame

        # Angular velocity: low-pass filter ω directly in body frame.
        ang_vel_body_filt = np.zeros(3)
        for i, lpf in enumerate(self.lpf_ang):
            ang_vel_body_filt[i] = lpf.filter(ang_vel_body_raw[i], dt)

        # ─────────────────────────────────────────────────────────
        #  Publish
        # ─────────────────────────────────────────────────────────
        self.pub_raw.publish(
            self._make_twist(stamp, msg.header.frame_id,
                             lin_vel_body_raw, ang_vel_body_raw))

        self.pub_filt.publish(
            self._make_plain_twist(lin_vel_body_filt, ang_vel_body_filt))

        # ── Advance state ─────────────────────────────────────────
        self.prev_pos  = curr_pos
        self.prev_quat = curr_quat
        self.prev_time = t_curr

    # ──────────────────────────────────────────────────────────────
    @staticmethod
    def _make_twist(stamp, frame_id: str,
                    lin: np.ndarray, ang: np.ndarray) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id
        msg.twist.linear.x  = float(lin[0])
        msg.twist.linear.y  = float(lin[1])
        msg.twist.linear.z  = float(lin[2])
        msg.twist.angular.x = float(ang[0])
        msg.twist.angular.y = float(ang[1])
        msg.twist.angular.z = float(ang[2])
        return msg

    @staticmethod
    def _make_plain_twist(lin: np.ndarray, ang: np.ndarray) -> Twist:
        """Build a plain (headerless) Twist, matching opti_vel's
        velocity_publisher publish contract on /rigid_body_velocity."""
        msg = Twist()
        msg.linear.x  = float(lin[0])
        msg.linear.y  = float(lin[1])
        msg.linear.z  = float(lin[2])
        msg.angular.x = float(ang[0])
        msg.angular.y = float(ang[1])
        msg.angular.z = float(ang[2])
        return msg


# ─────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = OptiTrackVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
