#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from custom_sensor_msgs.msg import IMUdata
from sensor_msgs.msg import Imu
import math
import time


class IMUtranslator(Node):

	def __init__(self):
		super().__init__('imu_translator')
		self.subscriber = self.create_subscription(Imu, 'imu', self.translate, 10)
		self.subscriber
            
		self.publisher_ = self.create_publisher(IMUdata, 'IMU', 10)
        
		self.alpha = 0.97
		self.gyroRoll = 0
		self.gyroPitch = 0
		self.gyroYaw = 0
		self.filterRoll = 0
		self.filterPitch = 0
		self.flag = True
		self.last = time.monotonic()
        

	def translate(self, msg):
		data = IMUdata()
		data.acc_x = msg.linear_acceleration.x
		data.acc_y = msg.linear_acceleration.y
		data.acc_z = msg.linear_acceleration.z
		data.gyro_x = msg.angular_velocity.x
		data.gyro_y = msg.angular_velocity.y
		data.gyro_z = msg.angular_velocity.z
		
		# acc data is too noisy to use on its own
		accPitch = math.degrees(math.atan2(data.acc_y , data.acc_z))
		accRoll = math.degrees(math.atan2(data.acc_x , data.acc_z))

		# complementary filter
		now = time.monotonic()
		dt = now - self.last
		self.last = now
		self.filterRoll = (self.alpha)*(self.filterRoll - data.gyro_y * dt) + (1-self.alpha)*(accRoll)
		self.filterPitch = (self.alpha)*(self.filterPitch + data.gyro_x  * dt) + (1-self.alpha)*(accPitch)

		data.pitch = math.radians(self.filterRoll) # TODO: use DMP, there coordinates are switched
		data.roll = math.radians(self.filterPitch)

		self.publisher_.publish(data)

def main(args=None):
    rclpy.init(args=args)

    translator = IMUtranslator()

    rclpy.spin(translator)

    translator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()