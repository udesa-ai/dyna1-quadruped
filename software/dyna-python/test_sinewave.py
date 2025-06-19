from classes.uart_bridge import UARTBridge
import time
import numpy as np

temp = []
temp_time = []

uart = UARTBridge()
uart.open()

T = 80

input('Press Enter to turn on Front Right motors...')
uart.send_on_off(state='on', motor='FRshoulder')
uart.send_on_off(state='on', motor='FRarm')
uart.send_on_off(state='on', motor='FRfoot')

input('Press Enter to start sine wave generation...')

# while loop until user interrupts with Ctrl+C
try:
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
           'BRfoot':0} # uart.read_uart()
    center = pos['FRfoot']
    t0 = time.time()
    while True:
        time.sleep(1/T)
        setpoint = center + np.sin(2 * np.pi * 0.25 * (time.time() - t0)) * 0.2
        pos['FRfoot'] = setpoint
        uart.send_positions(pos)

except KeyboardInterrupt:
       print('Exiting...')

uart.close()