from classes.brushless import BrushlessMotor
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
            
            self.brushless[leg+'shoulder'] = BrushlessMotor(leg_data["motor0"]["motorN"],
                                                            leg_data["motor0"]["iOffset"],
                                                            leg_data["motor0"]["axisID"],
                                                            leg_data["motor0"]["direction"],
                                                            leg+'shoulder')

            self.brushless[leg+'arm'] = BrushlessMotor(leg_data["motor1"]["motorN"],
                                                       leg_data["motor1"]["iOffset"],
                                                       leg_data["motor1"]["axisID"],
                                                       leg_data["motor1"]["direction"],
                                                       leg+'arm')

            self.brushless[leg+'foot'] = BrushlessMotor(leg_data["motor2"]["motorN"],
                                                        leg_data["motor2"]["iOffset"],
                                                        leg_data["motor2"]["axisID"],
                                                        leg_data["motor2"]["direction"],
                                                        leg+'foot')
        
        self.names = self.brushless.keys()
    
    def set_motor_encoders(self, motors):
        for motor in motors:
            self.brushless[motor].set_encoder_measurement(motors[motor])
            # print(f'{motor} set to {motors[motor]}')


    def get_motor_positions(self, motors):
        for motor in motors:
            pos = self.brushless[motor].get_position()
            motors[motor] = pos

    def get_angle(self, motors):
        for motor in motors:
            angle = self.brushless[motor].get_angle()/self.rrate
            motors[motor] = angle