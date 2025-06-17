from classes.motors_fake import Motors
from classes.uart_bridge import UARTBridge
import time
import struct

uart = UARTBridge()
uart.open()

motores = Motors()

pos = {'FRshoulder':0,
       'FRarm':0,
       'FRfoot':0,
       'FLshoulder':0,
       'FLarm':0,
       'FLfoot':0,
       'BLshoulder':0,
       'BLarm':0,
       'BLfoot':0,
       'BRshoulder':0,
       'BRarm':0,
       'BRfoot':0}

print('############################################################')

input('ready?')

# # Topic 1 expects 42 floats (IMU + 12 motores)
# test_floats = [i * 0.0 for i in range(42)]  # 0.0, 1.0, ..., 41.0
# payload = struct.pack('<42f', *test_floats)

# topic_id = 1
# seq = 0
# length = len(payload)

# frame = bytearray()
# frame += b'>>>'               # Sync
# frame.append(topic_id)        # Topic ID
# frame.append(seq)             # Sequence
# frame.append((length >> 8) & 0xFF)  # Length high
# frame.append(length & 0xFF)         # Length low
# frame += b'\x00\x00'          # Placeholder for CRC
# frame += payload              # Payload


# uart.ser.write(frame)
# uart.ser.flush()
# print(f"Sent test message ({len(frame)} bytes)")


while True:
	values = uart.read_uart()
	motores.set_motor_encoders(values)
	motores.get_angle(pos)

	for key, value in pos.items():
    		print(f'{key}: {value}')
	
	print()
	time.sleep(0.4)

uart.close()
