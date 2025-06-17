import argparse
from classes.uart_bridge import UARTBridge
import time
    
parser = argparse.ArgumentParser(description="Which odrive to reboot")

parser.add_argument('odrive', metavar='N', type=str, nargs='*')

args = parser.parse_args()

uart = UARTBridge()
uart.open()

odrives = uart.odrives

if len(args.odrive) == 0:
    print('Rebooting all odrive')
    for motor in odrives:
        input(f'Ready to reboot odrive of {motor}?')
        uart.send_reboot(odrives[motor])
        print('Done!')
else:
    for motor in args.odrive:
        if motor not in odrives:
            print(f'Odive {motor} not found')
            continue
        input(f'Ready to reboot odrive of {motor}?')
        uart.send_reboot(odrives[motor])
        print('Done!')

    
time.sleep(1)
uart.close()
print('Uart closed')

