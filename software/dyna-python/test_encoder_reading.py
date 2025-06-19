from classes.motors_fake import Motors
from classes.uart_bridge import UARTBridge
import time
import os
import sys

if sys.platform in ('linux', 'darwin'):
    CLEAR = 'clear'
elif sys.platform == 'win32':
    CLEAR = 'cls'
else:
    print('Platfrom not supported', file=sys.stderr)
    exit(1)


def clear_term() -> None:
    os.system(CLEAR)

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



# while loop until user interrupts with Ctrl+C
try:
       i = 0
       while True:
              values = uart.read_uart()
              motores.set_motor_encoders(values)
              motores.get_angle(pos)
              
              print(f'measurement {i}')
              print()
              j = 0
              for key, value in pos.items():
                     print(f'{key}: {round(value,2)}', end = '\t')
                     j += 1
                     if j % 3 == 0:
                           print()
                           print()
              
              print()
              i += 1
              time.sleep(0.4)
              clear_term()

except KeyboardInterrupt:
       print('Exiting...')

uart.close()
