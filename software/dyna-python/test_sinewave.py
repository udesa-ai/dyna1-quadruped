from classes.uart_bridge import UARTBridge
import time
import numpy as np
import argparse

# arpgarse to pass period and amplitude
parser = argparse.ArgumentParser(description="Sine wave generation for motors")
parser.add_argument('--period', type=float, default=80, help='Period of the sine wave in seconds')
parser.add_argument('--amplitude', type=float, default=0.2, help='Amplitude of the sine wave')
parser.add_argument('--frequency', type=float, default=0.25, help='Frequency of the sine wave in Hz')
args = parser.parse_args()

# Initialize variables
uart = UARTBridge()
uart.open()

T = args.period  # Period of the sine wave
amplitude = args.amplitude  # Amplitude of the sine wave
frequency = args.frequency  # Frequency of the sine wave in Hz

input('Press Enter to turn on Front Right motors...')
uart.send_on_off(state='on', motor='FRshoulder')
uart.send_on_off(state='on', motor='FRarm')
uart.send_on_off(state='on', motor='FRfoot')

input('Press Enter to start sine wave generation...')

# while loop until user interrupts with Ctrl+C
try:
    pos = uart.read_uart()
    center0 = pos['FRshoulder']['pos']
    center1 = pos['FRarm']['pos']
    center2 = pos['FRfoot']['pos']
    pos = {'FRshoulder':center0, 'FRarm':center1, 'FRfoot':center2, 'FLshoulder':0, 'FLarm':0, 'FLfoot':0, 'BLshoulder':0, 'BLarm':0, 'BLfoot':0, 'BRshoulder':0, 'BRarm':0, 'BRfoot':0 }
    t0 = time.time()
    while True:
        time.sleep(1/T)
        setpoint = np.sin(2 * np.pi * frequency * (time.time() - t0)) * amplitude
        pos['FRshoulder'] = setpoint + center0
        pos['FRarm'] = setpoint + center1
        pos['FRfoot'] = setpoint + center2
        uart.send_positions(pos)

except KeyboardInterrupt:
    uart.send_on_off('off')
    print('Exiting...')

uart.close()
