import rclpy
from rclpy.node import Node
import numpy as np
from joint_msgs.msg import Joints, NeuralInput
import time
import torch
import torch.nn as nn

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
        checkpoint = torch.load("/home/udesa/Documents/dyna-1/weigths/model_9999.pt", map_location=torch.device('cpu'))

        model_state_dict = checkpoint['model_state_dict']
        actor_state_dict = {k.replace('actor.', 'model.'): v for k, v in model_state_dict.items() if k.startswith('actor.')} 

        self.model = ActorMLP()
        self.model.load_state_dict(actor_state_dict)
        self.model.eval()

        # Actions
        self.actions = [0,0,0,0,0,0,0,0,0,0,0,0]

        #################################
        ########### Pub & Sub ###########
        #################################

        # Subscription a imu filtrada del bno055
        self.neural_sub = self.create_subscription(
            NeuralInput,
            'network_input',
            self.listener_neural,
            10
        )
        self.neural_sub

        # Publisher joint request
        self.pub_joint_angles = self.create_publisher(Joints, 'joint_requests', 1)

        print('Ready to receive neural input')


    def change_order(self, values, forwards = True):
        if forwards:
            return [values[6], values[9], values[0], values[3],
                    values[7], values[10], values[1], values[4],
                    values[8], values[11], values[2], values[5]]
        else:
            return [values[2], values[6], values[10],
                    values[3], values[7], values[11],
                    values[0], values[4], values[8],
                    values[1], values[5], values[9]]
        

    def listener_neural(self, msg):
        input_data = [0.0]*48
        input_data[0] = msg.base_lin_vel_x
        input_data[1] = msg.base_lin_vel_y
        input_data[2] = msg.base_lin_vel_z
        input_data[3] = msg.base_ang_vel_x
        input_data[4] = msg.base_ang_vel_y
        input_data[5] = msg.base_ang_vel_z
        input_data[6] = msg.projected_gravity_x
        input_data[7] = msg.projected_gravity_y
        input_data[8] = msg.projected_gravity_z
        input_data[9] = msg.x_velocity
        input_data[10] = msg.y_velocity
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
        
        input_data[36:48] = self.actions

        input_data = [float(value) for value in input_data]
        output = self.model(torch.tensor([input_data])).squeeze(0).tolist()
        self.actions = output
        self.real_actions = []
        temp_actions = self.change_order(self.actions, forwards = False)
        offsets = [0.0, -0.79, 1.5]
        for index, action in enumerate(temp_actions):
            self.real_actions.append(0.25*action + offsets[index%3])
        self.publishall([self.real_actions[0:3], self.real_actions[3:6], self.real_actions[6:9], self.real_actions[9:12]], max_check = False)


    def publishall(self, joint_angles, max_check = False):
        angles = [np.degrees(joint_angles[0][0]), np.degrees(joint_angles[0][1]), np.degrees(joint_angles[0][2]),
                  np.degrees(joint_angles[1][0]), np.degrees(joint_angles[1][1]), np.degrees(joint_angles[1][2]),
                  np.degrees(joint_angles[2][0]), np.degrees(joint_angles[2][1]), np.degrees(joint_angles[2][2]),
                  np.degrees(joint_angles[3][0]), np.degrees(joint_angles[3][1]), np.degrees(joint_angles[3][2])]

        ja_msg = Joints()
        
        ja_msg.flshoulder = angles[0]
        ja_msg.flarm = angles[1]
        ja_msg.flfoot = angles[2]

        ja_msg.frshoulder = angles[3]
        ja_msg.frarm = angles[4]
        ja_msg.frfoot = angles[5]

        ja_msg.blshoulder = angles[6]
        ja_msg.blarm = angles[7]
        ja_msg.blfoot = angles[8]

        ja_msg.brshoulder = angles[9]
        ja_msg.brarm = angles[10]
        ja_msg.brfoot = angles[11]
        
        t = self.get_clock().now()
        ja_msg.header.stamp = t.to_msg()

        ja_msg.check_max = max_check

        self.pub_joint_angles.publish(ja_msg)


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
