from classes.odrive_CAN import odriveCAN
from odrive.enums import *
import time


class BrushlessMotor:
    """
    BrushlessMotor class handles everything related to brushless motors controlled
    by an Odrive. To use this class the required odrive scripts must be downloaded
    from the documentation page. Please refer to this page for any Odrive API 
    related questions:  https://docs.odriverobotics.com/
    """
    
    def __init__(self, motorn, pos0, axisID, direc, name, odrive_instance=None):
        """
        To initialize the class two variables must be passed: 
        :param motorn: Motor number. Can be 0 or 1 depending on what Odrive
                       motor is being controlled
        :param pos0: Position that corresponds to 0 degrees. Measured in rotations
                     with respect to the motor 0 position (defined by startup or
                     by index search)
        :param odrive_instance: (optional) If an odrive instance was already
                                created for another active motor it must be
                                passed on initialization
        """
        self.name = name
        self.odrive = None
        # If there is no active Odrive instance a search is triggered
        if not odrive_instance:
            print('Finding an Odrive')
            self.odrive = odriveCAN('can0', motorn, axisID)
        else:
            self.odrive = odrive_instance
            self.odrive.get_axis(motorn, axisID)
        
        self.axis = self.odrive.axis1 if motorn else self.odrive.axis0
        self.motor = self.axis.motor
        self.encoder = self.axis.encoder
        self.controller = self.axis.controller
        self.mnumber = motorn
        self.pos0 = pos0
        self.direc = int(direc)
    
    def set_pos0(self, pos):
        self.pos0 = pos

    def index_search(self):
        """
        index_search can be used if the encoder attached to the motor has an
        index signal. It will block the code until an index is found
        """
        self.axis.requested_state(AXIS_STATE_ENCODER_INDEX_SEARCH)
        self.axis.wait_for_state(AXIS_STATE_ENCODER_INDEX_SEARCH)
        print('Index', self.mnumber, 'found!')
    

    def get_errors(self,):
        err_encoder = self.encoder.read_error()
        err_motor = self.motor.read_error()
        return err_encoder, err_motor

    def get_position(self):
        """
        get_position returns the position estimate (in turns with respect to 
        the local 0
        :returns position estimate
        """
        return self.direc * self.encoder.read_data()[0]
    

    def get_angle(self):
        """
        get_angle converts the position estimate to the angle estimate
        :returns angle estimate
        """
        return self.pos_to_angle(self.get_position())
    
    
    def set_max_current(self, current):
        """
        set_max_current sets the maximum motor current to be used. This setting
        resets to its default if the Odrive is rebooted. To change the default 
        current value the odrivetool must be invoked in the terminal. After 
        changing the current limit the new values must be saved to the Odrive memory.
        :param current: maximum current to be set. Measured in Amps
        """
        self.motor.config_current_lim(current)
    

    def set_control_mode(self, mode):
        """
        set_control_mode can be used to change the controller mode
        :param mode: Control mode to switch to
        """
        if mode == 'POSITION':
            self.controller.config_control_mode(CONTROL_MODE_POSITION_CONTROL)
        elif mode == 'VELOCITY':
            self.controller.config_control_mode(CONTROL_MODE_VELOCITY_CONTROL)
        elif mode == 'TORQUE':
            self.controller.config_control_mode(CONTROL_MODE_TORQUE_CONTROL)
        elif mode == 'VOLTAGE':
            self.controller.config_control_mode(CONTROL_MODE_VOLTAGE_CONTROL)
        else:
            print('Not a control mode')
    

    def set_pos_input_mode(self, mode):
        """
        set_pos_input_mode is used to change the position input mode
        :param mode: Input mode
        :param bandwidth: (optional) If filter mode is selected the filter
                          bandwidth can be selected. Measured in Hz
        """
        if mode == 'DIRECT':
            self.controller.config_input_mode(INPUT_MODE_PASSTHROUGH)
        elif mode == 'FILTER':
            self.controller.config_input_mode(INPUT_MODE_POS_FILTER)
        else:
            print('No imput mode matches selected input')
    

    def turn_on(self):
        """
        turn_on starts the motor closed loop control
        """
        self.axis.requested_state(AXIS_STATE_CLOSED_LOOP_CONTROL)
    

    def turn_off(self):
        """
        turn_off breaks the motor closed loop control. Motors will
        spin freely
        """
        self.axis.requested_state(1)
    

    def input_pos(self, position):
        """
        input_pos: Send a position request. The motor must be in closed loop
        and in position control to be able to move to the required position
        :param position: desired position. Measured in turns with respect to
                         the 0 turns position
        """
        self.controller.input_pos(self.direc * position)
    

    def input_ang(self, angle):
        """
        input_ang: Request a motor angle. The angle request is converted to 
        a position request via ang_to_pos()
        :param angle: desired angle measured with respect to the indicated 0
                      degree position. Measured in degrees
        """
        self.input_pos(self.ang_to_pos(angle))


    def ang_to_pos(self, angle):
        """
        ang_to_pos converts any angle measured with respect to the indicated
        0 position into a position value
        :param angle: angle to convert. Measured in degrees
        :return pos: position equivalent to input angle. Measured in turns
        """
        pos = angle/360 + self.pos0
        return pos


    def pos_to_angle(self, pos):
        """
        pos_to_angle converts any position with respect to the 0 turns mark
        into an angle measured with respect to the indicated 0 angle position
        :param pos: position to convert. Measured in turns
        :return angle: corresponding angle. Measured in degrees
        """
        angle = (pos - self.pos0)*360
        return angle


    def get_current(self):
        """
        get_current returns the measured current passing through the motor
        :return: motor current in Amps
        """
        return self.direc * self.motor.read_current()


    def set_torque_mode(self, mode):
        """
        set_torque_mode sets the desired torque input mode
        :param mode: type of torque input to receive
        :param rate: if the ramp mode is selected, the ramp rate can
                     be specified. Measured in Nm/s
        """
        if mode == 'DIRECT':
            self.controller.config_input_mode(INPUT_MODE_PASSTHROUGH)
        elif mode == 'RAMP':
            self.controller.config_input_mode(INPUT_MODE_TORQUE_RAMP)
        else:
            print('No imput mode matches selected input')
        

    def input_torque(self, torque):
        """
        input_torque sends the requested torque to the odrive
        :param torque: torque request. Measured in Nm
        """
        self.controller.input_torque(self.direc * torque)
    

    def get_torque(self):
        """
        get_torque returns the calculated torque based on the torque constant
        and the current. TODO: return torque based on torque vs. current curve
        :return motor torque in Nm
        """
        return self.get_current()*self.motor.config_torque_constant
    

    def get_velocity_rpm(self):
        """
        get_velocity_rpm returns the estimated RPM of the motor
        :return motor velocity in rpm
        """
        return self.direc * self.encoder.read_data()[1]*60
    
    def get_ang_velrpm(self,):
        pos, vel = self.encoder.read_data()
        ang = self.pos_to_angle(self.direc * pos)
        return ang, self.direc * vel*60

################################################################


