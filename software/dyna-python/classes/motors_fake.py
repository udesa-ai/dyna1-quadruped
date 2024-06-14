import time

class Motors:

    def __init__(self,):

        self.rrate = 9

        self.brushless = {}

        for leg in ['FR', 'FL', 'BL', 'BR']:       
            self.brushless[leg+'shoulder'] = 'motor'
            self.brushless[leg+'arm'] = 'motor'
            self.brushless[leg+'foot'] = 'motor'
        
        self.names = self.brushless.keys()

    def reboot_odrive(self, motors=None):
        print('Done!')
    

    def get_errors(self,):
        print('all cool')


    def find_index(self, motors):
        print('done')


    def get_motor_positions(self, motors, n=1):
        pass


    def set_zero_positions(self, motors):
        print('done')

    def calibrate_pos0(self,):
        print('All motors calibrated and config file updated!')

    def get_angle(self, motors, n=1):
        motors['FLshoulder'] = -10
        motors['FLarm'] = -50
        motors['FLfoot'] = 70
        motors['FRshoulder'] = 10
        motors['FRarm'] = -50
        motors['FRfoot'] = 70
        motors['BLshoulder'] = -10
        motors['BLarm'] = -50
        motors['BLfoot'] = 70
        motors['BRshoulder'] = -10
        motors['BRarm'] = -50
        motors['BRfoot'] = 70


    def turn_on(self, motors = None):
        print('motors on')
    

    def turn_off(self, motors = None):
        print('motors off')
    

    def go_to_ang(self, motors):
        pass


    def set_position_control(self, motors, filter):
        print('Done setting position control')

    def get_current(self, motors):
        pass


    def get_torques(self, motors):
        pass    

    def set_torque_control(self, motors, ramp):
        print('Done setting torque control')


    def go_to_torque(self, motors):
        pass


    def set_max_currents(self, motors):
        print('Done setting max current')


    def get_angle_velrpm(self,motors):
        pass


    def get_vel_rpm(self, motors):
        pass

    def calibrate_motors(self, motors):
        print('All motors calibrated and config file updated!')