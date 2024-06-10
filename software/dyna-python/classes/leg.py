from classes.leg_math import for_k, f2t, w2v
from classes.brushless_CAN import BrushlessMotor
import time
import math
from classes.leg_kinematics import LegK

class Leg:
    """
    The Leg class handles everything related to the Dyna-1 leg. All cartesian
    positions are measured in meters. From a top perspective the X axis is 
    forward, the Y axis is to the left and the Z axis is out of the page
    """

    def __init__(self, name, leg_data, odrv_center = None):
        """
        When initializing the user can select wether or not to search for an index.
        Thanks to the Odrive internal memory, if the index is already found, as long
        as its not rebooted it doesn't have to be searched for again
        :param index_search: (optional) boolean to set if index search is desired
        """
        self.name = name

        self.L1 = 0.0435 # Servo link length
        self.L2 = 0.190 # Link 1 length
        self.L3 = 0.194 # link 2 length

        self.rrate = 9 # reduction rate
        
        self.motor0 = BrushlessMotor(leg_data["motor0"]["motorN"],
                                     leg_data["motor0"]["iOffset"],
                                     leg_data["motor0"]["axisID"],
                                     leg_data["motor0"]["direction"],
                                     odrive_instance = odrv_center)
            
        self.motor1 = BrushlessMotor(leg_data["motor1"]["motorN"],
                                     leg_data["motor1"]["iOffset"],
                                     leg_data["motor1"]["axisID"],
                                     leg_data["motor1"]["direction"])

        self.motor2 = BrushlessMotor(leg_data["motor2"]["motorN"],
                                     leg_data["motor2"]["iOffset"],
                                     leg_data["motor2"]["axisID"],
                                     leg_data["motor2"]["direction"],
                                     odrive_instance = self.motor1.odrive)

        self.legKinematics = LegK(legtype = leg_data["legType"],
                                  shoulder_length = self.L1,
                                  elbow_length = self.L2,
                                  wrist_length = self.L3)
        
        self.motors = [self.motor0, self.motor1, self.motor2]


    def find_index(self, axis = [0, 1, 2]):
        for axi in axis:
            input(f'Ready to search for index of axis {axi} of {self.name}?')
            self.motors[axi].index_search()


    def IK(self, pos):
        """
        IK takes the cartesian position of the end effector and returns the
        corresponding angles
        :param pos: cartesian position in meters with respect to the leg axis
                    in a list of the form [x, y, z]
        :return: corresponding angles in degrees in a list [t0, t1, t2]
        """
        return self.legKinematics.solveIK(pos)
    

    def FK(self, ang):
        """
        FK takes the leg angles and returns the corresponding cartesian position
        :param ang: leg angles in degrees in a list [t0, t1, t2]
        :return: cartesian position in meters with respect to the leg axis in
                 a list [x, y, z]
        """
        return self.legKinematics.solveFK(ang)

    def get_motor_positions(self, n=1):
        """
        get_angle returns all the motor angles in degrees. An optional input can
        set how many measurements to average over.
        :param n: (optional) number of measurements to average over
        :return: measured leg angles in degrees in the form of a list [t0, t1, t2]
        """
        t0 = 0
        t1 = 0
        t2 = 0
        for i in range(1,n+1): # calculating the average
            t0 += (self.motor0.get_position() - t0)/i # brushless angle is divided by reduction rate
            t1 += (self.motor1.get_position() - t1)/i # brushless angle is divided by reduction rate
            t2 += (self.motor2.get_position() - t2)/i # brushless angle is divided by reduction rate
        return [t0, t1, t2]

    def set_zero_positions(self, positions):
        for motor, pos in zip(self.motors, positions):
            motor.set_pos0(pos)

    def get_angle(self, n=1):
        """
        get_angle returns all the motor angles in degrees. An optional input can
        set how many measurements to average over.
        :param n: (optional) number of measurements to average over
        :return: measured leg angles in degrees in the form of a list [t0, t1, t2]
        """
        t0 = 0
        t1 = 0
        t2 = 0
        for i in range(1,n+1): # calculating the average
            t0 += (self.motor0.get_angle()/self.rrate - t0)/i # brushless angle is divided by reduction rate
            t1 += (self.motor1.get_angle()/self.rrate - t1)/i # brushless angle is divided by reduction rate
            t2 += (self.motor2.get_angle()/self.rrate - t2)/i # brushless angle is divided by reduction rate
        return [t0, t1, t2]
    

    def get_position(self, n=1):
        """
        get_position returns the current cartesian position of the end effector. Can
        optionally average over various measurements
        :param n: (optional) number of measurements to average over
        :return: list of cartesian positions in meters ([x, y, z])
        """
        return self.FK(self.get_angle(n))
    

    def turn_on(self,):
        """
        turn_on turns on all motors
        """
        self.motor0.turn_on()
        self.motor1.turn_on()
        self.motor2.turn_on()
    

    def turn_off(self):
        """
        turn_off turns off all motors
        """
        self.motor0.turn_off()
        self.motor1.turn_off()
        self.motor2.turn_off()
    

    def go_to_pos(self, pos):
        """
        go_to_pos requests that the leg end effector goes to the desired
        cartesian position. Measured in meters with respect to the leg axis
        :param pos: cartesian position in meters in a list [x, y, z]
        """
        angles = self.IK(pos)
        self.go_to_ang(angles)
    

    def go_to_ang(self, angles):
        """
        go_to_ang sends the angle requests to the corresponding motors. Depending
        on the axee parameter the request can be for the three motors or just the
        servo motor
        :param angles: desired angles in degrees. If axee is 3 then it must be a 
                       list in the form [t0, t1, t2], else it can be just a number
        """
        self.motor0.input_ang(angles[0]*self.rrate) # brushless angle is multiplied by reduction rate
        self.motor1.input_ang(angles[1]*self.rrate) # brushless angle is multiplied by reduction rate
        self.motor2.input_ang(angles[2]*self.rrate) # brushless angle is multiplied by reduction rate


    def set_position_control(self, filter, freq=1):
        """
        set_position_control makes sure that switching from any other form of control
        to position control does not cause any problems. To do so firt it makes the 
        position input to be the same as the current position. It then makes the input
        mode direct so that the input is directly passed as the current setpoint. After
        doing so the filter can be activated depending on the filter boolean. Finally it
        switches the control mode to position to enable position control
        :param filter: boolean to set wether the position control is applied for both
                       motors or just one in particular (True: filtered, False: direct)
        :param freq: (optional) second order filter frequency in Hz
        :return: returns 0 if something goes wrong
        """
        self.motor0.input_pos(self.motor0.get_position())
        self.motor1.input_pos(self.motor1.get_position())
        self.motor2.input_pos(self.motor2.get_position())
        time.sleep(0.001)
        self.motor0.set_pos_input_mode('DIRECT')
        self.motor1.set_pos_input_mode('DIRECT')
        self.motor2.set_pos_input_mode('DIRECT')
        time.sleep(0.001)
        if filter:
            self.motor0.set_pos_input_mode('FILTER', freq)
            self.motor1.set_pos_input_mode('FILTER', freq)
            self.motor2.set_pos_input_mode('FILTER', freq)

        self.motor0.set_control_mode('POSITION')
        self.motor1.set_control_mode('POSITION')
        self.motor2.set_control_mode('POSITION')


    def get_current(self):
        """
        get_current returns the current passing through the motors
        :return currents: measured currents of the motors returned as a list in
                          the form of [current0, current 1, current 2]
        """
        current0 = self.motor1.get_current()
        current1 = self.motor1.get_current()
        current2 = self.motor2.get_current()
        return [current0, current1, current2]


    def get_torques(self):
        """
        get_torques returns the current leg torques. These are the motor torques
        multiplied by the reduction rate to get axis torque
        :return torques: axis torques in a list [torque0, torque 1, torque 2]
        """
        torq0 = self.motor0.get_torque()*self.rrate
        torq1 = self.motor1.get_torque()*self.rrate
        torq2 = self.motor2.get_torque()*self.rrate
        return [torq0, torq1, torq2]
    

    def set_z_spring(self, pos_desired, spring_constant, damping_constant, ramp, rate=1):
        """
        set_z_spring prepares the leg for torque control and spring control. Similarly to \
        set_position_control it firts sets the torque input to be the current torque. Then \
        the torque input is passed directly to the setpoint. If a torque ramp setting is \
        selected then the input mode is set to ramp. Finally the leg is set to torque \
        control mode. If nothing else is done the leg should stay in place (asuming it was \
        still in the first place).
        :param pos_desired: This is the desired cartesian position (in meters with respect \
                            to the leg axis). The virtual springs will have this as the \
                            neutral point
        :param spring_constant: virtual spring constant for every direction in N/m. In the \
                                form of a list [kx, ky, kz]
        :param damping_constant: virtual damping constant for every direction in N.s/m. In \
                                 the form of a list [bx, by, bz]
        :param ramp: boolean to fix wether the torque is ramped to the input torque or \
                     passed directly (True: ramped torque, False: direct torque)
        :param rate: (optional) torque ramp rate in Nm/s
        """
        self.pos_desired = pos_desired
        self.spring_constant = spring_constant
        self.damping_constant = damping_constant
        self.springf = True

        self.motor0.input_torque(self.motor0.get_torque())
        self.motor1.input_torque(self.motor1.get_torque())
        self.motor2.input_torque(self.motor2.get_torque())
        time.sleep(0.001)
        self.motor0.set_torque_mode('DIRECT')
        self.motor1.set_torque_mode('DIRECT')
        self.motor2.set_torque_mode('DIRECT')
        time.sleep(0.001)
        if ramp:
            self.motor0.set_torque_mode('RAMP', rate)
            self.motor1.set_torque_mode('RAMP', rate)
            self.motor2.set_torque_mode('RAMP', rate)
        
        self.motor0.set_control_mode('TORQUE')
        self.motor1.set_control_mode('TORQUE')
        self.motor2.set_control_mode('TORQUE')


    def spring(self):
        """
        spring actuates the leg as if it was a virtual spring. For it to work \
        properly it must be called periodically at a reasonable frequency. The \
        required torques are calculated following the example of a cartesian \
        impedance controller in https://arxiv.org/pdf/1910.00093.pdf
        :return torques: if a spring is defined then the function returns the \
                         requested torques in a list [T0, T1, T2]
        """
        if self.springf:
            ang, pos, vel = self.get_ang_pos_velXYZ()

            force0 = self.spring_constant[0]*(self.pos_desired[0] - pos[0])
            force1 = self.spring_constant[1]*(self.pos_desired[1] - pos[1])
            force2 = self.spring_constant[2]*(self.pos_desired[2] - pos[2])
            
            damping0 = self.damping_constant[0]*vel[0]
            damping1 = self.damping_constant[1]*vel[1]
            damping2 = self.damping_constant[2]*vel[2]

            torque = f2t(self, ang, [force0-damping0, force1-damping1, force2-damping2])

            self.motor0.input_torque(-torque[0]/self.rrate) # desired torque is divided by reduction rate
            self.motor1.input_torque(-torque[1]/self.rrate) # desired torque is divided by reduction rate
            self.motor2.input_torque(-torque[2]/self.rrate) # and reversed due to motor default direction
            return [torque[0], torque[1], torque[2]], pos
        else:
            print('No spring defined')
    

    def set_max_currents(self, value):
        """
        set_max_currents sets the maximum current for both motors
        :param value: maximum current in amps
        """
        self.motor0.set_max_current(value)
        self.motor1.set_max_current(value)
        self.motor2.set_max_current(value)


    def get_vel_xyz(self):
        """
        get_vel_xyz returns the current velocities (in m/s) for all the cartesian
        directions of the leg
        :return velocities: velocities in m/s in a list in the form of [vx, vy, vz]
        """
        angles = self.get_angle()
        w0 = self.motor0.get_velocity_rpm()*math.pi/(30*self.rrate)
        w1 = self.motor1.get_velocity_rpm()*math.pi/(30*self.rrate)
        w2 = self.motor2.get_velocity_rpm()*math.pi/(30*self.rrate)
        w = [w0, w1, w2]
        v = w2v(self, angles, w)
        vx = v[0]
        vy = v[1]
        vz = v[2]
        return [vx, vy, vz]
    

    def get_ang_pos_velXYZ(self, n=1):
        """
        get_position returns the current cartesian position of the end effector. Can
        optionally average over various measurements
        :param n: (optional) number of measurements to average over
        :return: list of cartesian positions in meters ([x, y, z])
        """
        angles, velrpm = self.get_angle_velrpm()
        pos = self.FK(angles)
        w0 = velrpm[0]*math.pi/(30*self.rrate)
        w1 = velrpm[1]*math.pi/(30*self.rrate)
        w2 = velrpm[2]*math.pi/(30*self.rrate)
        w = [w0, w1, w2]
        velxyz = w2v(self, angles, w)
        return angles, pos, velxyz

    
    def get_angle_velrpm(self,):
        m0ang, m0vel = self.motor0.get_ang_velrpm()
        m1ang, m1vel = self.motor1.get_ang_velrpm()
        m2ang, m2vel = self.motor2.get_ang_velrpm()

        t0 = m0ang/self.rrate # brushless angle is divided by reduction rate
        t1 = m1ang/self.rrate # brushless angle is divided by reduction rate
        t2 = m2ang/self.rrate # brushless angle is divided by reduction rate

        v = [m0vel, m1vel, m2vel]

        return [t0, t1, t2], v


    def get_vel_rpm(self):
        """
        get_vel_rpm returns the current velocities (in RPM) for all motors
        :return velocities: velocities in RPM in a list in the form of [w0, w1, w2]
        """
        w0 = self.motor0.get_velocity_rpm()
        w1 = self.motor1.get_velocity_rpm()
        w2 = self.motor2.get_velocity_rpm()
        return [w0, w1, w2]