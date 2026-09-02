import rclpy
from rclpy.node import Node
import numpy as np
from joint_msgs.msg import Joints, NeuralInput, NeuralInputComplete
import time
import torch
import torch.nn as nn
import csv
import os
from collections import deque
from datetime import datetime
from pathlib import Path


NET_INPUT_DT = 0.02

GRAVITY = 9.81
G_WORLD = np.array([0.0, 0.0, -GRAVITY])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y**2 + z**2),     2 * (x * y - z * w),       2 * (x * z + y * w)],
        [    2 * (x * y + z * w), 1 - 2 * (x**2 + z**2),       2 * (y * z - x * w)],
        [    2 * (x * z - y * w),     2 * (y * z + x * w),   1 - 2 * (x**2 + y**2)],
    ])


class Kalman:
    def __init__(
        self,
        dt_ref: float,
        q_scale: float = 0.001,
        r_scale: float = 30.0,
        adaptive_gain: float = 80.0,
        q_init: np.ndarray | None = None,
    ):
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) if q_init is None else q_init
        self.P = np.eye(4) * 0.1
        self.Q = np.eye(4) * q_scale
        self.R = np.eye(3) * r_scale
        self.dt_ref = dt_ref
        self.adaptive_gain = adaptive_gain

    def predict(self, gyro: np.ndarray, dt: float) -> None:
        if dt <= 0:
            return
        omega_quat = np.array([0.0, gyro[0], gyro[1], gyro[2]])
        q_dot = 0.5 * quat_multiply(self.q, omega_quat)
        self.q = quat_normalize(self.q + q_dot * dt)

        wx, wy, wz = gyro
        Omega = np.array([
            [0.0, -wx, -wy, -wz],
            [wx,   0.0,  wz, -wy],
            [wy,  -wz,  0.0,  wx],
            [wz,   wy, -wx,  0.0],
        ])
        F = np.eye(4) + 0.5 * dt * Omega
        self.P = F @ self.P @ F.T + self.Q * (dt / self.dt_ref)

    def update(self, accel_corrected: np.ndarray) -> None:
        R_body = quat_to_rotmat(self.q)
        g_expected = R_body.T @ G_WORLD

        accel_mag = np.linalg.norm(accel_corrected)
        accel_norm = accel_corrected / (accel_mag + 1e-8)
        h_norm = g_expected / (np.linalg.norm(g_expected) + 1e-8)
        innov = accel_norm - h_norm

        w, x, y, z = self.q

        H = 2.0 * np.array([
            [ y, -z,  w, -x],
            [-x, -w, -z, -y],
            [0.0, 2 * x, 2 * y, 0.0],
        ])

        deviation = abs(accel_mag - GRAVITY) / GRAVITY
        R_eff = self.R * (1.0 + self.adaptive_gain * deviation**2)

        S = H @ self.P @ H.T + R_eff
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = np.zeros((4, 3))

        self.q = quat_normalize(self.q + K @ innov)
        self.P = (np.eye(4) - K @ H) @ self.P


# Define the model architecture
class ActorMLP(nn.Module):
    def __init__(self):
        super(ActorMLP, self).__init__()
        self.model = nn.Sequential(
            nn.Linear(48, 128),
            nn.ELU(alpha=1.0),
            nn.Linear(128, 128),
            nn.ELU(alpha=1.0),
            nn.Linear(128, 128),
            nn.ELU(alpha=1.0),
            nn.Linear(128, 12)
        )

    def forward(self, x):
        return self.model(x)

class NeuralNet(Node):

    def __init__(self):
        super().__init__('NeuralNet')
        print('Starting Neural Net controller')

        ############ Variables ###############
        self.declare_parameter('model_path','')
        self.model_path = self.get_parameter("model_path").value
        checkpoint = torch.load("/home/dynabot/ppo_policy.pt") #, map_location=torch.device('cpu'))

        model_state_dict = checkpoint['model_state_dict']
        actor_state_dict = {k.replace('actor.', 'model.'): v for k, v in model_state_dict.items() if k.startswith('actor.')} 

        self.model = ActorMLP()
        self.model.load_state_dict(actor_state_dict)
        self.model.eval()

        self.kf = Kalman(dt_ref=NET_INPUT_DT)
        self.q = self.kf.q  # Initial quaternion

        # Actions
        self.actions = [0,0,0,0,0,0,0,0,0,0,0,0]

        self.action_delay = 5
        self.action_buffer = deque([[0.0]*12 for _ in range(self.action_delay)], maxlen=self.action_delay)

        #################################
        ########### Pub & Sub ###########
        #################################

        # Subscribe to net inputs
        self.neural_sub = self.create_subscription(
            NeuralInput,
            'network_input',
            self.listener_neural,
            10
        )
        self.neural_sub

        # Publisher joint request
        self.pub_joint_angles = self.create_publisher(Joints, 'joint_requests', 1)

        # Publisher net input for debugging
        self.pub_net_input = self.create_publisher(NeuralInputComplete, 'net_input_debugging', 1)

        # ROS info print
        self.get_logger().info('Neural Net controller initialized')

        self.iteration_i = 0
        self.iteration_a = 0

        # Initialize CSV for recording observations
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        data_dir = Path.home() / "neural_net_data"
        data_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = data_dir / f"observations_{timestamp}.csv"
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header: timestamp + 48 observation values + 12 actions
        header = ['timestamp']
        header += [f'obs_{i}' for i in range(48)]
        header += [f'action_{i}' for i in range(12)]
        self.csv_writer.writerow(header)
        self.csv_file.flush()

        self.get_logger().info(f'Observations will be saved to: {self.csv_path}')

    def quat_to_rotmat(self, q):
        w, x, y, z = q
        return np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - z*w),       2*(x*z + y*w)],
            [    2*(x*y + z*w),   1 - 2*(x**2 + z**2),     2*(y*z - x*w)],
            [    2*(x*z - y*w),       2*(y*z + x*w),   1 - 2*(x**2 + y**2)]
        ])

    def change_order(self, values, forwards = True):
        if forwards:
            return [values[6], values[9], values[3], values[0],
                    values[7], values[10], values[4], values[1],
                    values[8], values[11], values[5], values[2]]
        else:
            return [values[3], values[7], values[11],
                    values[2], values[6], values[10], 
                    values[0], values[4], values[8],
                    values[1], values[5], values[9]]
    

    def listener_neural(self, msg):
        input_data = [0.0]*48
        input_data[0] = msg.base_lin_vel_x
        input_data[1] = msg.base_lin_vel_y
        input_data[2] = msg.base_lin_vel_z
        

        gyro = np.array([msg.base_ang_vel_x,
                     msg.base_ang_vel_y,
                     msg.base_ang_vel_z])
    

        accel_corrected = np.array([msg.projected_gravity_x,
                        msg.projected_gravity_y,
                        msg.projected_gravity_z]) * GRAVITY

        # Update quaternion
        self.kf.predict(gyro, NET_INPUT_DT)
        self.kf.update(accel_corrected)
        self.q = self.kf.q

        # Compute projected gravity from quaternion
        # Rotation matrix from quaternion:
        R = self.quat_to_rotmat(self.q)
        g_proj = R @ np.array([0.0, 0.0, -1.0])  # gravity in body frame

        input_data[3:6] = gyro
        input_data[6:9] = g_proj
        # print(g_proj)

        # input_data[6] = msg.projected_gravity_x
        # input_data[7] = msg.projected_gravity_y
        # input_data[8] = msg.projected_gravity_z

        vx = msg.x_velocity
        vy = msg.y_velocity

        norm = np.sqrt(vx**2 + vy**2)

        if norm > 1.0:
            vx /= norm
            vy /= norm

        input_data[9] = vx
        input_data[10] = vy
        
        input_data[11] = msg.w_rate

        input_joints = [0.0]*12
        input_joints[0] = msg.joint_angle_0
        input_joints[1] = msg.joint_angle_1
        input_joints[2] = msg.joint_angle_2
        input_joints[3] = msg.joint_angle_3
        input_joints[4] = msg.joint_angle_4
        input_joints[5] = msg.joint_angle_5
        input_joints[6] = msg.joint_angle_6
        input_joints[7] = msg.joint_angle_7
        input_joints[8] = msg.joint_angle_8
        input_joints[9] = msg.joint_angle_9
        input_joints[10] = msg.joint_angle_10
        input_joints[11] = msg.joint_angle_11
        input_data[12:24] = self.change_order(input_joints)

        input_vels = [0.0]*12
        input_vels[0] = msg.joint_velocity_0
        input_vels[1] = msg.joint_velocity_1
        input_vels[2] = msg.joint_velocity_2
        input_vels[3] = msg.joint_velocity_3
        input_vels[4] = msg.joint_velocity_4
        input_vels[5] = msg.joint_velocity_5
        input_vels[6] = msg.joint_velocity_6
        input_vels[7] = msg.joint_velocity_7
        input_vels[8] = msg.joint_velocity_8
        input_vels[9] = msg.joint_velocity_9
        input_vels[10] = msg.joint_velocity_10
        input_vels[11] = msg.joint_velocity_11
        input_data[24:36] = self.change_order(input_vels)
        
        input_data[36:48] = self.action_buffer.popleft()

        input_data = [float(value) for value in input_data]
        # log input data
        # self.print_input(input_data)
        # Publish the input data for debugging
        self.publish_input(input_data)
        output = self.model(torch.tensor([input_data])).squeeze(0).tolist()
        self.actions = output
        self.action_buffer.append(output)

        # Save observations to CSV
        timestamp = time.time()
        row = [f"{timestamp:.6f}"] + [f"{val:.6f}" for val in input_data] + [f"{val:.6f}" for val in output]
        self.csv_writer.writerow(row)
        self.csv_file.flush()
        self.real_actions = []
        temp_actions = self.change_order(self.actions, forwards = False)
        offsets = [0.0, -0.79, 1.5]
        for index, action in enumerate(temp_actions):
            self.real_actions.append(0.25*action + offsets[index%3])
        # # log output actions every 50 iterations
        # if self.iteration_a == 50:
        #     self.iteration_a = 0
        #     self.get_logger().info(f'Output Actions: {[f"{action:.3f}" for action in self.actions]}')
        # else:
        #     self.iteration_a += 1
        # Publish the actions
        self.publishall([self.real_actions[0:3], self.real_actions[3:6], self.real_actions[6:9], self.real_actions[9:12]])

    def publish_input(self, input_data):
        net_input_msg = NeuralInputComplete()
        net_input_msg.base_lin_vel_x = input_data[0]
        net_input_msg.base_lin_vel_y = input_data[1]
        net_input_msg.base_lin_vel_z = input_data[2]
        net_input_msg.base_ang_vel_x = input_data[3]
        net_input_msg.base_ang_vel_y = input_data[4]
        net_input_msg.base_ang_vel_z = input_data[5]
        net_input_msg.projected_gravity_x = input_data[6]
        net_input_msg.projected_gravity_y = input_data[7]
        net_input_msg.projected_gravity_z = input_data[8]
        net_input_msg.x_velocity = input_data[9]
        net_input_msg.y_velocity = input_data[10]
        net_input_msg.w_rate = input_data[11]

        for i in range(12):
            setattr(net_input_msg, f'joint_angle_{i}', input_data[12 + i])
            setattr(net_input_msg, f'joint_velocity_{i}', input_data[24 + i])
            setattr(net_input_msg, f'previous_action_{i}', input_data[36 + i])
    

        self.pub_net_input.publish(net_input_msg)

    def print_input(self, input_data):
        # Print every 50 iterations
        if self.iteration_i == 50:
            self.iteration_i = 0
        else:
            self.iteration_i += 1
            return
        # log input data with 3 decimal places
        text = f'''Input Data: 
        \tBase Linear Velocity: x={input_data[0]:.2f}, y={input_data[1]:.2f}, z={input_data[2]:.2f}
        \tBase Angular Velocity: x={input_data[3]:.2f}, y={input_data[4]:.2f}, z={input_data[5]:.2f}
        \tProjected Gravity: x={input_data[6]:.2f}, y={input_data[7]:.2f}, z={input_data[8]:.2f}
        \tDesired Velocities: x={input_data[9]:.2f}, y={input_data[10]:.2f}, w={input_data[11]:.2f}
        \tJoint Angles: {[f"{data:.3f}" for data in input_data[12:24]]}
        \tJoint Velocities: {[f"{data:.3f}" for data in input_data[24:36]]}
        \tPrevious Actions: {[f"{data:.3f}" for data in input_data[36:48]]}'''
        self.get_logger().info(text)
        

    def publishall(self, joint_angles):
        angles = [np.degrees(joint_angles[0][0]), np.degrees(joint_angles[0][1]), np.degrees(joint_angles[0][2]),
                  np.degrees(joint_angles[1][0]), np.degrees(joint_angles[1][1]), np.degrees(joint_angles[1][2]),
                  np.degrees(joint_angles[2][0]), np.degrees(joint_angles[2][1]), np.degrees(joint_angles[2][2]),
                  np.degrees(joint_angles[3][0]), np.degrees(joint_angles[3][1]), np.degrees(joint_angles[3][2])]

        ja_msg = Joints()
        
        ja_msg.frshoulder = angles[0]
        ja_msg.frarm = angles[1]
        ja_msg.frfoot = angles[2]

        ja_msg.flshoulder = angles[3]
        ja_msg.flarm = angles[4]
        ja_msg.flfoot = angles[5]

        ja_msg.blshoulder = angles[6]
        ja_msg.blarm = angles[7]
        ja_msg.blfoot = angles[8]

        ja_msg.brshoulder = angles[9]
        ja_msg.brarm = angles[10]
        ja_msg.brfoot = angles[11]
        
        t = self.get_clock().now()
        ja_msg.header.stamp = t.to_msg()

        self.pub_joint_angles.publish(ja_msg)

    def destroy_node(self):
        """Close CSV file when node shuts down."""
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f'Observations saved to: {self.csv_path}')
        super().destroy_node()


def main(args=None):
    
    rclpy.init(args=args)

    neural_net = NeuralNet()
    # atexit.register(real_interface.rescue_data)
    rclpy.spin(neural_net)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    neural_net.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
