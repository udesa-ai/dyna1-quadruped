from classes.quad_kinematics import QuadModel
from classes.motors import Motors
import numpy as np

class Dyna:

    def __init__(self, ):
        self.robotMotors = Motors()

        self.quadKine = QuadModel(shoulder_length=0.050,
                                   elbow_length=0.190,
                                   wrist_length=0.194,
                                   hip_x=0.455,
                                   hip_y=0.197,
                                   foot_x=0.455,
                                   foot_y=0.197 + 0.1,
                                   height=0.190+0.194,
                                   com_offset=0)

    
    def get_dict(self,):
        return dict.fromkeys(self.robotMotors.names, 0)
   
    
    def get_angles(self,):
        angle_data = self.get_dict()
        self.robotMotors.get_angle(angle_data)
        return angle_data

    def get_currents(self,):
        currents = self.get_dict()
        self.robotMotors.get_current(currents)
        return currents

    def get_xyz(self,):
        angles_m = self.get_angles()
        joint_angles = np.zeros((4,3))
        for index, leggo in enumerate(self.quadKine.Legs):
            angles = np.array([angles_m[leggo + 'shoulder'], angles_m[leggo + 'arm'], angles_m[leggo + 'foot']])*np.pi/180
            joint_angles[index,:] = angles
        
        xyz = self.quadKine.FK(joint_angles)

        return xyz

    def reboot_motors(self,):
        self.robotMotors.reboot_odrive()


    def calibrate_motors(self,LEGS = None):
        self.robotMotors.calibrate_pos0(LEGS)
    

    def turn_on(self,):
        self.robotMotors.turn_on(self.get_dict())

    def turn_off(self,):
        self.robotMotors.turn_off(self.get_dict())
    
    def set_max_current(self, currents):
        self.robotMotors.set_max_currents(currents)
    
    def go_to_ang(self, angles):
        self.robotMotors.go_to_ang(angles)
