import serial
import time
import struct

class UARTBridge:

    def __init__(self, port = '/dev/ttyTHS1', baudrate = 460800):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.odrives = {'FRshoulder':0,
                        'FRarm':1,
                        'FLarm':4,
                        'BLshoulder':6,
                        'BLarm':7,
                        'BRarm':10}

        self.motors = {'FRshoulder':0,
                       'FRarm':1,
                       'FRfoot':2,
                       'FLshoulder':3,
                       'FLarm':4,
                       'FLfoot':5,
                       'BLshoulder':6,
                       'BLarm':7,
                       'BLfoot':8,
                       'BRshoulder':9,
                       'BRarm':10,
                       'BRfoot':11}

        self.SYNC = b'>>>'

    def open(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=1)

    def close(self):
        if self.ser:
            self.ser.close()

    def calculate_crc(self, data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                if (crc & 0x8000):
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    def send_on_off(self, state = None, motor = None):
        if state == None and motor == None:
            for m in self.motors.values():
                self.send_motor_state(m, 1)
                time.sleep(0.1)
                self.send_motor_state(m, 0)
                time.sleep(0.1)
        else:
            state = 1 if state == 'on' else 0
            if motor == None:
                for m in self.motors.values():
                    self.send_motor_state(m, state)
                    time.sleep(0.1)
            else:
                self.send_motor_state(self.motors[motor], state)

    def send_motor_state(self, motor, state):
        # message is now two bytes, first is motor number, second is state (0 or 1)

        # payload is two bytes: motor number and state
        # both motor number and state are integers (uint8_t) and are the arguments of the function
        payload = [
            motor & 0xFF,  # motor number (uint8_t)
            state & 0xFF   # state (uint8_t)    
        ]

        frame = []
        frame.append(ord('>'))
        frame.append(ord('>'))
        frame.append(ord('>'))

        # id is 7 sent as one byte
        id = 0x07
        frame.append(id)

        frame.append(0x00)

        # Longitud del payload
        frame.append((2 >> 8) & 0xFF)  # len_high
        frame.append(2 & 0xFF)         # len_low

        # Placeholder para el CRC
        frame.append(0x00)  # CRC high
        frame.append(0x00)  # CRC low

        # Agregar el payload
        frame.extend(payload)

        # Calcular el CRC desde topic_id hasta fin de payload
        data_for_crc = bytes(frame[9:])  # desde index 9 hasta el final
        crc = self.calculate_crc(data_for_crc)

        frame[7] = (crc >> 8) & 0xFF
        frame[8] = crc & 0xFF

        # Convertir a bytes para enviar (string en C++, bytes en Python)
        frame_bytes = bytes(frame)
        self.ser.write(frame_bytes)

    def send_reboot(self, motors):
        numbers = [0]*12
        numbers[motors] = 1
        self.send_bits(6, numbers)
    
    def send_bits(self, id, values):
        bitfield = 0x0000
        for i in range(12):
            bitfield |= (values[i] << i)

        # Lista de bytes para el payload (2 bytes para uint16)
        payload = [
            bitfield & 0xFF,         # byte bajo
            (bitfield >> 8) & 0xFF   # byte alto
        ]

        frame = []
        frame.append(ord('>'))
        frame.append(ord('>'))
        frame.append(ord('>'))

        frame.append(id)

        frame.append(0x00)

        # Longitud del payload
        frame.append((2 >> 8) & 0xFF)  # len_high
        frame.append(2 & 0xFF)         # len_low

        # Placeholder para el CRC
        frame.append(0x00)  # CRC high
        frame.append(0x00)  # CRC low

        # Agregar el payload
        frame.extend(payload)

        # Calcular el CRC desde topic_id hasta fin de payload
        data_for_crc = bytes(frame[9:])  # desde index 9 hasta el final
        crc = self.calculate_crc(data_for_crc)

        frame[7] = (crc >> 8) & 0xFF
        frame[8] = crc & 0xFF

        # Convertir a bytes para enviar (string en C++, bytes en Python)
        frame_bytes = bytes(frame)
        self.ser.write(frame_bytes)

    def handle_payload(self, payload):
        floats = struct.unpack('<42f', payload)  # 42 floats, little-endian

        imu = {
            'accel': floats[0:3],
            'gyro': floats[3:6],
        }

        motors = {}
        for i in range(12):
            pos = floats[6 + i]
            vel = floats[18 + i]
            curr = floats[30 + i]
            motors[list(self.motors.keys())[i]] = {'pos': pos, 'vel': vel, 'curr': curr}

        return motors

    def read_uart(self, ):
        self.ser.flushInput()
        buffer = bytearray()

        while True:
            data = self.ser.read(64)
            buffer.extend(data)

            while len(buffer) >= 9:
                sync_index = buffer.find(self.SYNC)
                if sync_index == -1:
                    buffer.clear()
                    break

                if len(buffer) < sync_index + 9:
                    break  # wait for more bytes

                topic_id = buffer[sync_index + 3]
                length = (buffer[sync_index + 5] << 8) | buffer[sync_index + 6]
                frame_len = 3 + 1 + 1 + 2 + 2 + length

                if len(buffer) < sync_index + frame_len:
                    break  # incomplete frame

                frame = buffer[sync_index:sync_index + frame_len]
                del buffer[:sync_index + frame_len]

                payload = frame[9:]

                return self.handle_payload(payload)
