from classes.brushless_CAN import BrushlessMotor
import json
import time

class Motors:

    def __init__(self,):
        with open('config.json','r') as f:
            data = json.load(f)
        
        self.data = data

        self.rrate = 9

        self.brushless = {}

        for leg in ['FR', 'FL', 'BL', 'BR']:
            leg_data = data[leg]

            if leg in ['FR', 'BL']:
                odrv_center = None
            
            self.brushless[leg+'shoulder'] = BrushlessMotor(leg_data["motor0"]["motorN"],
                                                            leg_data["motor0"]["iOffset"],
                                                            leg_data["motor0"]["axisID"],
                                                            leg_data["motor0"]["direction"],
                                                            leg+'shoulder',
                                                            odrive_instance = odrv_center)
            
            odrv_center = self.brushless[leg+'shoulder'].odrive

            self.brushless[leg+'arm'] = BrushlessMotor(leg_data["motor1"]["motorN"],
                                                       leg_data["motor1"]["iOffset"],
                                                       leg_data["motor1"]["axisID"],
                                                       leg_data["motor1"]["direction"],
                                                       leg+'arm')
            
            odrv_m12 = self.brushless[leg+'arm'].odrive

            self.brushless[leg+'foot'] = BrushlessMotor(leg_data["motor2"]["motorN"],
                                                        leg_data["motor2"]["iOffset"],
                                                        leg_data["motor2"]["axisID"],
                                                        leg_data["motor2"]["direction"],
                                                        leg+'foot',
                                                        odrive_instance = odrv_m12)
        
        self.names = self.brushless.keys()

    def reboot_odrive(self, motors=None):
        if motors == None:
            motors = ['FRshoulder','FRarm','FLarm','BRshoulder','BRarm','BLarm']
        
        for motor in motors:
            motor = self.brushless[motor]
            input(f'Ready to reboot odrive of {motor.name}?')
            motor.odrive.reboot()
        print('Done!')
    

    def get_errors(self,):
        col_width = 20
        print('name'.ljust(col_width) + 'encoder error'.ljust(col_width) + 'motor error')
        print('-'*3*col_width)
        for name, motor in self.brushless.items():
            encoder, mot = motor.get_errors()
            print(name.ljust(col_width) + str(encoder).ljust(col_width) + str(mot))



    def find_index(self, motors):
        for motor in motors:
            input(f'Ready to search for index of {motor}?')
            self.brushless[motor].index_search()


    def get_motor_positions(self, motors, n=1):
        for motor in motors:
            pos = 0
            for i in range(100):
                temp = self.brushless[motor].get_position()
            for i in range(1,n+1):
                temp = self.brushless[motor].get_position()
                print(temp)
                pos += (temp - pos)/i 
            motors[motor] = pos


    def set_zero_positions(self, motors):
        for motor, pos0 in motors.items():
            self.brushless[motor].set_pos0(pos0)

    def calibrate_pos0(self,LEGS = None):
        if LEGS == None:
            LEGS = ['FR', 'FL', 'BL', 'BR']
        print(f'Will calibrate {LEGS}')
        for leg in LEGS:
            input(f'Ready to calibrate {leg}?')
            leggo = {f'{leg}shoulder': 0, f'{leg}arm':0, f'{leg}foot':0}
            self.get_motor_positions(leggo, n=10)
            for index, mot in enumerate(leggo):
                c_angle = self.data[leg][f'motor{index}']["calibration_angle"]
                pos0 = leggo[mot] - c_angle*self.rrate/360
                self.data[leg][f'motor{index}']["iOffset"] = pos0
                print(leggo[mot], c_angle, pos0)
                leggo[mot] = pos0
                
            self.set_zero_positions(leggo)
        with open("config.json",'w') as file:
            json.dump(self.data, file, indent=4)
        print('All motors calibrated and config file updated!')

    def get_angle(self, motors, n=1):
        for motor in motors:
            angle = 0
            for i in range(1,n+1):
                angle += (self.brushless[motor].get_angle()/self.rrate - angle)/i 
            motors[motor] = angle


    def turn_on(self, motors = None):
        if motors == None:
            motors = self.names
        
        for motor in motors:
            self.brushless[motor].turn_on()
    

    def turn_off(self, motors = None):
        if motors == None:
            motors = self.names
        
        for motor in motors:
            self.brushless[motor].turn_off()
    

    def go_to_ang(self, motors):
        for motor, angle in motors.items():
            self.brushless[motor].input_ang(angle*self.rrate)


    def set_position_control(self, motors, filter):
        for motor in motors:
            self.brushless[motor].input_pos(self.brushless[motor].get_position())

        time.sleep(0.001)

        for motor in motors:
            self.brushless[motor].set_pos_input_mode('DIRECT')

        time.sleep(0.001)

        if filter:
            for motor in motors:
                self.brushless[motor].set_pos_input_mode('FILTER')

        for motor in motors:
            self.brushless[motor].set_control_mode('POSITION')


    def get_current(self, motors):
        for motor in motors:
            motors[motor] = self.brushless[motor].get_current()


    def get_torques(self, motors):
        for motor in motors:
            motors[motor] = self.brushless[motor].get_torque()*self.rrate
    

    def set_torque_control(self, motors, ramp):
        for motor in motors:
            self.brushless[motor].input_torque(self.brushless[motor].get_torque())
        
        time.sleep(0.001)

        for motor in motors:
            self.brushless[motor].set_torque_mode('DIRECT')

        time.sleep(0.001)

        if ramp:
            for motor in motors:
                self.brushless[motor].set_torque_mode('RAMP')
        
        for motor in motors:
            self.brushless[motor].set_control_mode('TORQUE')


    def go_to_torque(self, motors):
        for motor, torque in motors.items():
            self.brushless[motor].input_torque(torque/self.rrate)


    def set_max_currents(self, motors):
        for motor, value in motors.items():
            self.brushless[motor].set_max_current(value)


    def get_angle_velrpm(self,motors):
        for motor in motors:
            mang, mvel = self.brushless[motor].get_ang_velrpm()
            motors[motor] = [mang/self.rrate, mvel/self.rrate]


    def get_vel_rpm(self, motors):
        for motor in motors:
            motors[motor] = self.brushless[motor].get_velocity_rpm()


    def calibrate_motors(self, motors):
        for motor_name in motors:
            if 'shoulder' in motor_name:
                mnumber = 0
            elif 'arm' in motor_name:
                mnumber = 1
            else:
                mnumber = 2
            motor = self.brushless[motor_name]
            input(f'Ready to calibrate {motor.name}?')
            position = motor.get_position()
            c_angle = self.data[motor_name[:2]][f'motor{mnumber}']["calibration_angle"]
            pos0 = position - c_angle*self.rrate/360
            self.data[motor_name[:2]][f'motor{mnumber}']["iOffset"] = pos0
            motor.set_pos0(pos0)
        with open("config.json",'w') as file:
            json.dump(self.data, file)
        print('All motors calibrated and config file updated!')