import time


class BrushlessMotor:
    
    def __init__(self, motorn, pos0, axisID, direc, name):
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
        # If there is no active Odrive instance a search is triggered
        
        self.mnumber = motorn
        self.pos0 = pos0
        self.direc = int(direc)
        self.position = 0
        self.velocity = 0
    
    def set_pos0(self, pos):
        self.pos0 = pos
    
    def set_encoder_measurement(self, measurements):
        self.position = measurements['pos']
        self.velocity = measurements['vel']

    def get_position(self):
        """
        get_position returns the position estimate (in turns with respect to 
        the local 0
        :returns position estimate
        """
        return self.direc * self.position
    

    def get_angle(self):
        """
        get_angle converts the position estimate to the angle estimate
        :returns angle estimate
        """
        return self.pos_to_angle(self.get_position())


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


    def get_velocity_rpm(self):
        """
        get_velocity_rpm returns the estimated RPM of the motor
        :return motor velocity in rpm
        """
        return self.direc * self.velocity*60


################################################################


