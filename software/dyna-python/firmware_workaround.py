import argparse
from classes.uart_bridge import UARTBridge
import time

uart = UARTBridge()
uart.open()


print('############################################################')
input('ready?')

uart.send_on_off('on')

time.sleep(1)

uart.send_on_off('off')

print('Done!')
uart.close()
