from classes.CAN import CAN
import cantools
from odrive.enums import *
import time
from collections import deque
import numpy as np

db = cantools.database.load_file("classes/odrive-cansimple.dbc")

class odriveCAN():
    
    def __init__(self, port, axisN, axisID):
        self.can = CAN.getInstance(port)
        self.get_axis(axisN, axisID)

    def reboot(self, ):
        data = db.encode_message('Axis0_Reboot',{})
        msg = self.can.message(db.get_message_by_name('Axis0_Reboot').frame_id | self.axis0.axisID << 5, False, data)
        self.can.send(msg)
        self.axis0.wait_for_heartbeat()
    
    def get_axis(self, axisN, axisID):
        if axisN == 0:
            self.axis0ID = axisID
            self.axis0 = axis(0, self.axis0ID, self.can)
            self.axis0.wait_for_heartbeat()
        else:
            self.axis1ID = axisID
            self.axis1 = axis(1, self.axis1ID, self.can)
            self.axis1.wait_for_heartbeat()

        print(f'Axis {axisN} with ID {axisID} found!')
        
    
class axis:

    def __init__(self, axisN, axisID, can):
        self.axisN = axisN
        self.axisID = axisID
        self.motor = motor(axisID, can, axisN) #, self.axisN, sensor)
        self.controller = controller(axisID, can, axisN)
        self.encoder = encoder(axisID, can, axisN) #, self.axisN, sensor)
        self.can = can
    
    def requested_state(self, state):
        # msg = db.get_message_by_name('Axis0_Set_Axis_State')
        # data = msg.encode({'Axis0_Axis_Requested_State': state})
        data = db.encode_message('Axis0_Set_Axis_State',{'Axis_Requested_State':state})
        msg = self.can.message(db.get_message_by_name('Axis0_Set_Axis_State').frame_id | self.axisID << 5, False, data)
        self.can.send(msg)


    def wait_for_heartbeat(self,):
        while True:
            msg = self.can.recv()
            if msg.arbitration_id == ((self.axisID << 5) | db.get_message_by_name('Axis0_Heartbeat').frame_id):
                print(f'Heartbeat for axisID {self.axisID} received!')
                break

    def wait_for_state(self, start_state):
        state_flag = False
        while True:
            msg = self.can.recv()
            if msg.arbitration_id == ((self.axisID << 5) | db.get_message_by_name('Axis0_Heartbeat').frame_id):
                current_state = db.decode_message('Axis0_Heartbeat', msg.data)['Axis_State']
                if not state_flag and current_state == AxisState(start_state).name:
                    state_flag = True
                if state_flag and current_state == 'IDLE':
                    print("Axis has returned to Idle state.")
                    break

class encoder:

    def __init__(self, axisID, can, axisN): #, n, sensor):
        self.axisN = axisN
        self.pos_estimate = 0
        self.vel_estimate = 0
        self.axisID = axisID
        self.msgID = (axisID << 5) | db.get_message_by_name('Axis0_Get_Encoder_Estimates').frame_id
        self.errorID = (axisID << 5) | db.get_message_by_name('Axis0_Get_Encoder_Error').frame_id
        self.can = can
    
    def update_estimates(self, msg):
        self.pos_estimate = db.decode_message('Axis0_Get_Encoder_Estimates', msg.data)['Pos_Estimate']
        self.vel_estimate = db.decode_message('Axis0_Get_Encoder_Estimates', msg.data)['Vel_Estimate']

    def request_estimates(self,):
        datum = db.encode_message('Axis0_Get_Encoder_Estimates', {'Pos_Estimate':0.0, 'Vel_Estimate':0.0})
        msg = self.can.message(self.msgID, False, datum, True)
        self.can.send(msg)
    

    def request_error(self,):
        datum = db.encode_message('Axis0_Get_Encoder_Error', {'Encoder_Error':0.0})
        msg = self.can.message(self.errorID, False, datum, True)
        self.can.send(msg)


    def read_error(self,):
        self.request_error()
        msg = self.can.recv()
        while msg == None or msg.arbitration_id != self.errorID:
            msg = self.can.recv()
        data = db.decode_message('Axis0_Get_Encoder_Error', msg.data)['Encoder_Error']
        return data


    def read_data(self,):
        msg = self.can.recv()
        while msg == None or msg.arbitration_id != self.msgID:
            msg = self.can.recv()
        self.update_estimates(msg)
        return self.pos_estimate, self.vel_estimate

    

class controller:

    def __init__(self, axisID, can, axisN):
        self.axisN = axisN
        self.axisID = axisID
        self.can = can
        self.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.input_mode = INPUT_MODE_PASSTHROUGH
    
    def config_mode(self):
        data = db.encode_message('Axis0_Set_Controller_Mode', {'Input_Mode': self.input_mode, 'Control_Mode':self.control_mode})
        ID = db.get_message_by_name('Axis0_Set_Controller_Mode').frame_id | self.axisID << 5
        msg = self.can.message(ID, False, data)
        self.can.send(msg)

    def config_control_mode(self, mode):
        self.control_mode = mode
        self.config_mode()
    
    def config_input_mode(self, mode):
        self.input_mode = mode
        self.config_mode()
    
    
    def input_pos(self, position, vel_ff=0.0, torque_ff=0.0):
        data_dict = {'Input_Pos':position}
        data_dict['Vel_FF'] = vel_ff
        data_dict['Torque_FF'] = torque_ff
        data = db.encode_message('Axis0_Set_Input_Pos', data_dict)
        ID = db.get_message_by_name('Axis0_Set_Input_Pos').frame_id | self.axisID << 5
        msg = self.can.message( ID, False, data)
        self.can.send(msg)
    
    def input_torque(self, torque):
        data = db.encode_message('Axis0_Set_Input_Torque', {'Input_Torque':torque})
        ID = db.get_message_by_name('Axis0_Set_Input_Torque').frame_id | self.axisID << 5
        msg = self.can.message( ID, False, data)
        self.can.send(msg)


class motor:

    def __init__(self, axisID, can, axisN):
        self.axisN = axisN
        self.axisID = axisID
        self.msgID = db.get_message_by_name('Axis0_Get_Iq').frame_id | self.axisID << 5
        self.errorID = db.get_message_by_name('Axis0_Get_Motor_Error').frame_id | self.axisID << 5
        self.can = can
        self.Iq_measured = 0
        self.config_torque_constant = 0.0344088


    def config_current_lim(self, current):
        data = db.encode_message('Axis0_Set_Limits', {'Current_Limit':current, 'Velocity_Limit':1000})
        ID = db.get_message_by_name('Axis0_Set_Limits').frame_id | self.axisID << 5
        msg = self.can.message(ID, False, data)
        self.can.send(msg)

    def request_current_measurement(self):
        data = db.encode_message('Axis0_Get_Iq', {'Iq_Measured':1.0, 'Iq_Setpoint':0.0})
        msg = self.can.message(self.msgID, False, data, True)
        self.can.send(msg)
    
    def request_error(self):
        data = db.encode_message('Axis0_Get_Motor_Error', {'Motor_Error':0.0})
        msg = self.can.message(self.errorID, False, data, True)
        self.can.send(msg)

    def update_current(self, msg):
        self.Iq_measured = db.decode_message('Axis0_Get_Iq', msg.data)['Iq_Measured']
    
    def read_current(self,):
        self.request_current_measurement()
        msg = self.can.recv()
        while msg == None or msg.arbitration_id != self.msgID:
            msg = self.can.recv()
        self.update_current(msg)
        return self.Iq_measured

    def read_error(self,):
        self.request_error()
        msg = self.can.recv()
        while msg == None or msg.arbitration_id != self.errorID:
            msg = self.can.recv()
        data = db.decode_message('Axis0_Get_Motor_Error', msg.data)['Motor_Error']
        return data
