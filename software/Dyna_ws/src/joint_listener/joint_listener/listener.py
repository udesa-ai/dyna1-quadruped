import rclpy
from rclpy.node import Node
from joint_msgs.msg import Joints, OdriveData
from sensor_msgs.msg import Imu
import os
import time
import atexit


class Jointlistener(Node):

    def __init__(self):
        super().__init__('joint_listener')

        self.declare_parameter('dataLocation', '')

        self.data_sub = self.create_subscription(
            OdriveData,
            '/joint_data',
            self.data_callback,
            1000)
        self.data_sub  # prevent unused variable warning

        self.request_sub = self.create_subscription(
            Joints,
            '/joint_requests',
            self.request_callback,
            1000)
        self.request_sub  # prevent unused variable warning

        self.data_IMU = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            1000)
        self.data_IMU  # prevent unused variable warning

        self.angles = []
        self.velocities = []
        self.currents = []
        self.commands = []
        self.imu = []

        timestr = time.strftime("%Y%m%d-%H%M%S")
        
        self.dataLoc = self.get_parameter('dataLocation').get_parameter_value().string_value
        self.directoryLoc = os.path.join(self.dataLoc,timestr)
        self.controlLocation = os.path.join(self.directoryLoc, 'control.csv')
        self.measuredLocation = os.path.join(self.directoryLoc, 'measured.csv')
        self.measuredVelocity = os.path.join(self.directoryLoc, 'measured_velocity.csv')
        self.currentLocation = os.path.join(self.directoryLoc, 'currents.csv')
        self.imulocation = os.path.join(self.directoryLoc, 'imu.csv')

    def data_callback(self, msg):
        self.angles.append([msg.angles.header.stamp.sec + msg.angles.header.stamp.nanosec/1000000000,
                          msg.angles.flshoulder, msg.angles.flarm, msg.angles.flfoot,
                          msg.angles.frshoulder, msg.angles.frarm, msg.angles.frfoot,
                          msg.angles.blshoulder, msg.angles.blarm, msg.angles.blfoot,
                          msg.angles.brshoulder, msg.angles.brarm, msg.angles.brfoot])
        
        self.velocities.append([msg.angles.header.stamp.sec + msg.angles.header.stamp.nanosec/1000000000,
                          msg.velocities.flshoulder, msg.velocities.flarm, msg.velocities.flfoot,
                          msg.velocities.frshoulder, msg.velocities.frarm, msg.velocities.frfoot,
                          msg.velocities.blshoulder, msg.velocities.blarm, msg.velocities.blfoot,
                          msg.velocities.brshoulder, msg.velocities.brarm, msg.velocities.brfoot])
        
        self.currents.append([msg.angles.header.stamp.sec + msg.angles.header.stamp.nanosec/1000000000,
                          msg.currents.flshoulder, msg.currents.flarm, msg.currents.flfoot,
                          msg.currents.frshoulder, msg.currents.frarm, msg.currents.frfoot,
                          msg.currents.blshoulder, msg.currents.blarm, msg.currents.blfoot,
                          msg.currents.brshoulder, msg.currents.brarm, msg.currents.brfoot])    
        
    def request_callback(self, msg):
        self.commands.append([msg.header.stamp.sec + msg.header.stamp.nanosec/1000000000,
                              time.monotonic(),
                          msg.flshoulder, msg.flarm, msg.flfoot,
                          msg.frshoulder, msg.frarm, msg.frfoot,
                          msg.blshoulder, msg.blarm, msg.blfoot,
                          msg.brshoulder, msg.brarm, msg.brfoot,
                          msg.motion, msg.movement])
    
    def imu_callback(self, msg):
        self.imu.append([msg.header.stamp.sec + msg.header.stamp.nanosec/1000000000,
                          msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                          msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    
    def save(self):
        os.makedirs(self.directoryLoc,exist_ok=True)
        with open(self.controlLocation,'w') as file:
            header = 'time_start,time_end,flshoulder,flarm,flfoot,frshoulder,frarm,frfoot,blshoulder,blarm,blfoot,brshoulder,brarm,brfoot,motion,movement\n'
            file.write(header)
            for dataset in self.commands:
                msg = ''
                for i in range(16):
                    msg += f'{dataset[i]},'
                msg = msg[:-1] + '\n'
                file.write(msg)
        
        header = 'time_start,flshoulder,flarm,flfoot,frshoulder,frarm,frfoot,blshoulder,blarm,blfoot,brshoulder,brarm,brfoot\n'
        with open(self.currentLocation,'w') as file:
            file.write(header)
            for dataset in self.currents:
                msg = ''
                for i in range(13):
                    msg += f'{dataset[i]},'
                msg = msg[:-1] + '\n'
                file.write(msg)
            

        with open(self.measuredLocation,'w') as file:
            file.write(header)
            for dataset in self.angles:
                msg = ''
                for i in range(13):
                    msg += f'{dataset[i]},'
                msg = msg[:-1] + '\n'
                file.write(msg)
        
        with open(self.measuredVelocity,'w') as file:
            file.write(header)
            for dataset in self.velocities:
                msg = ''
                for i in range(13):
                    msg += f'{dataset[i]},'
                msg = msg[:-1] + '\n'
                file.write(msg)
        
        with open(self.imulocation,'w') as file:
            file.write('time_start,av_x,av_y,av_z,la_x,la_y,la_z\n')
            for dataset in self.imu:
                msg = ''
                for i in dataset:
                    msg += f'{i},'
                msg = msg[:-1] + '\n'
                file.write(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Jointlistener()
    atexit.register(minimal_subscriber.save)
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
