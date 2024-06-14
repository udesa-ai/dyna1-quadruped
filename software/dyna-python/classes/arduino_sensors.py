import serial
import time


class ArduinoSensor:

    def __init__(self,port = '/dev/ttyTHS2', baud_rate = 9600):
        self._port = port
        self.baud_rate = baud_rate
        self.ser = serial.Serial(self._port, self.baud_rate)
        print('Waiting for serial to complete connection')
        time.sleep(1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print('Done waiting for serial')
        
    
    def close(self,):
        self.ser.close()


    def start_recording(self, duration = 5):
        self.altura = []
        self.timer = []
        self.Az = []
        self.ser.write(b'\x01')
        t0 = time.monotonic()
        t1 = time.monotonic()
        while t1 - t0 < duration:
            t1 = time.monotonic()
            if self.ser.in_waiting>=4:
                ul1, ul2, z1, z2 = self.ser.read(4)
                distance = (ul1*(2**8) + ul2)/10
                # X = uint_2_int(x1, x2)
                # Y = uint_2_int(y1, y2)
                Z = uint_2_int(z1, z2)          
                # accelx = (X / 4096) * 9.81
                # accely = (Y / 4096) * 9.81
                accelz = ((Z - 4096) / 4096) * 9.81
                #print(f'Distance : {distance}, AZ : {accelz}', flush = True)
                
                self.altura.append(distance)
                self.timer.append(t1)
                self.Az.append(accelz)
            time.sleep(0.0005)

        self.ser.write(b'\x00')


def uint_2_int(n1, n2):
    num = (n1*2**8 + n2)
    if num > 2**15:
        num = 2**16 - num
    return num