import argparse
from classes.uart_bridge import UARTBridge
import time

uart = UARTBridge()
uart.open()


print('############################################################')
input('ready?')

debugging = False

uart.send_on_off('off')

uart.send_on_off('on')

time.sleep(20)

uart.send_on_off('off')

for i in range(12):
    uart.send_on_off('on', i)
    if debugging:
        time.sleep(1)
        while uart.ser.in_waiting > 0:
            data = uart.ser.read(uart.ser.in_waiting)
            print(" ".join(f"{b:02X}" for b in data))
    time.sleep(1)
    uart.send_on_off('off', i)
    if debugging:
        time.sleep(1)
        while uart.ser.in_waiting > 0:
            data = uart.ser.read(uart.ser.in_waiting)
            print(" ".join(f"{b:02X}" for b in data))
    time.sleep(1)
print('Done!')
uart.close()
