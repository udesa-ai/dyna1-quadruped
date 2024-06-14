import pigpio
import time


def ang2pwm(ang):
    """
    ang2pwm converts the angular request into a pwm duty cycle to be sent to
    the servo motor. For a 0 angle request the motor will move to the middle
    of its movement span. The pwm duty cicle is measured in percentage, with
    0 corresponding to 0% and 1e6 corresponding to 100%. Considering that the
    signal period is 20 ms, the center signal is about 1.4 ms
    """
    pwm = int(ang * ((122000 - 70250)/90) + 70250)
    return pwm


def spi2ang(data):
    """
    spi2ang converts the spi signal received from the encoder into the
    measured angle. The center value is currently fitted to the Dyna-1 leg
    teststand setup
    """
    cero = 14016.01551724138
    ang = (data - cero) * (90 / (14288 - 10146))
    return ang


class ServoMotor:
    """
    The ServoMotor class handles everything related to the servo motor. Many
    control functions and angle functions require an encoder mounted on the 
    motor to function properly. In this case the class is fitted to work with
    the magnetic encoder AS5047P that communicates through spi
    """
    
    def __init__(self, pwmpin, center, use_spi=True):
        """
        When initialized te class requires the pwm pin number, the motor center
        value dictated by placement of joint, and the state of the encoder. If
        using spi the encoder must be connected to pins 10 (MOSI), 9 (MISO), 11
        (SCLK), 8 (SPI0 CE0) and ground. In the case of the AS5047P the MOSI pin
        can be left unconected.
        :param pwmpin: Raspberry pwm pin number in BCM. Must be either 12 or 13
                       This code however was only tested with pin 12
        :param center: Servo center offset in degrees
        :param use_spi: (optional) Defines if encoder with spi is used or not
        """
        self.pi = pigpio.pi()
        self.pwmpin = pwmpin
        self.center = center
        
        self.use_spi = use_spi
        
        if use_spi:
            """
            Flag:
            bits 0-1 set spi mode: currently in mode 1 with CPOL = 0 and CPHA = 1
            bits 2-4 set CEx active low (default) or 1 for active high. Currently
                    set to active low
            bits 5-7 reserve CEx GPIO if 0. Currently CE0 is reserved for SPI
            bit 8 is 0 for main SPI
            bit 9 is 0 for non 3-wire connection
            bits 10-13 are not used in non 3-wire settings
            bit 14 is not used in main spi mode
            bit 15 is not used in main spi mode
            bits 16-21 are not used in main spi mode

            Spi channel set to 0
            Baud rate is set as 76000 bits per second

            For more information visit the pigpio documentation:
                https://abyz.me.uk/rpi/pigpio/python.html#spi_open
            """
            self.flags = 0b0000000000000011000001
            self.spi = self.pi.spi_open(0,76000,self.flags)
        
        self.error_old = None
        self.P = None
        self.D = None
        self.I = None
        
        self.alpha = 0.1
        self.s0 = 0
        
        self.error_sum = 0

        self.prev = None
        self.tprev = None
        self.v0 = 0
        self.valpha = 0.3
    
    
    def turn_on(self, position):
        """
        turn_on begins sending the pwm signal to the servo motor. The signal
        frequency is set to 50 to allow for a 20 ms signal period
        :param position: desired servo position for startup. Measured in degrees
                         with respect to the servo center defined in when
                         initializing the object
        """
        real_position = position + self.center
        if abs(real_position) > 90:
            print('Requested position out of range!')
        else:
            self.pi.hardware_PWM(self.pwmpin, 50, ang2pwm(real_position))
    

    def go_to(self, position):
        """
        go_to sets the pwm duty cycle so that the servo goes to the desired 
        angular position
        :param  position: desired position in degrees measured with respect to the 
                          pre-defined center position
        """
        real_position = position + self.center
        if abs(real_position) > 90:
            print('Requested position out of range!')
        else:
            self.pi.hardware_PWM(self.pwmpin, 50, ang2pwm(real_position))
    
    
    def go_to_pid(self, position, tdif, rangle = False):
        """
        go_to_pid is similar to go_to but uses a PID closed loop to control the
        motor position. For it to work properly it must be called at a regular
        frecuency.
        :param position: Desired angular position (in degrees) of the servo 
                         measured with respect to the encoder signal
        :param tdif: time diference between this call and the previous function
                     call. Measured in seconds
        :param rangle: (optional) boolean to define if the function should return
                       the measured angle or not
        :return angle: (optional) based on rangle the function can return the 
                       measured angle
        """
        mod_position = position
        angle = self.get_angle()
        error = position - angle
        if self.P:
            mod_position += self.P * error
        if self.D:
            # For the D control an alpha filter is aplied to the noisy data 
            if not self.error_old:
                self.error_old = error
            xt = (error - self.error_old)/tdif
            s1 = self.alpha*xt + (1-self.alpha)*self.s0
            mod_position += self.D*s1
            self.error_old = error
            self.s0 = s1
        if self.I:
            self.error_sum += error * tdif
            mod_position += self.I * self.error_sum
        
        angle = mod_position + self.center
        if angle > 90:
            print('servo request too high!')
        elif angle < -90:
            print('servo request too low!')
        else:
            self.pi.hardware_PWM(self.pwmpin, 50, ang2pwm(angle))
        
        if rangle:
            return angle
    
    
    def turn_off(self):
        """
        turn_off sends a 0 duty cycle signal to the motor that lets it move freely
        """
        # self.pi.write(self.pwmpin, 0)
        self.pi.hardware_PWM(self.pwmpin, 0, 0)
    
    
    def close_motor(self):
        """
        close_motor closes the spi instance and the pi object instance
        """
        if self.use_spi:
            self.pi.spi_close(self.spi)
        self.pi.stop()
    
    
    def get_angle(self):
        """
        get_angle returns the measured angle from the spi encoder. To read the
        angle first two bytes must be written
        :return: servo angle in degrees
        """
        self.pi.spi_write(self.spi, b'\x00\x00')
        (check, angle) = self.pi.spi_read(self.spi,2)
        data = int.from_bytes(angle,'big') & 0x3FFF
        angle = spi2ang(data)
        
        return angle
    
    
    def set_P_constant(self, value):
        """
        set_P_constant sets the proportional control constant. If it is not set
        the PID control will not include a proportional value
        :param value: P constant
        """
        self.P = value
    

    def set_D_constant(self, value):
        """
        set_D_constant sets the derivative constant of the PID control loop. If
        it is not set then the PID control will not have a derivative component
        :param value: D constant
        """
        self.D = value
    

    def set_I_constant(self, value):
        """
        set_I_constant sets the integra constant of the PID control loop. If
        it is not set then the PID control will have no integrative component
        :param value: I constant
        """
        self.I = value
    

    def get_velocity_rpm(self):
        """
        get_velocity_rpm calculates and returns an approximate value of the servo
        velocity in rpm based on its angle measurements. For more accurate readings
        it should be called periodically. For the firt call it will return 0 rpm
        :return v1: servo velocity in RPM
        """
        ang = self.get_angle()
        time_now = time.monotonic()
        if not self.prev:
            v1 = 0
        else:
            # alpha filtering to smoothen the curve
            tdif = time_now - self.tprev
            ang_dif = ang - self.prev
            vel = ang_dif/tdif
            vel_rpm = vel/6
            v1 = self.valpha*vel_rpm + (1-self.valpha)*self.v0
            self.v0 = v1
            
        self.prev = ang
        self.tprev = time_now
        
        return v1



