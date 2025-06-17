import argparse
from classes.uart_bridge import UARTBridge
import time
    
parser = argparse.ArgumentParser(description="Which odrive to reboot")

parser.add_argument('odrive', metavar='N', type=str, nargs='*')

args = parser.parse_args()

debugging = False

uart = UARTBridge()
uart.open()

odrives = uart.odrives

if len(args.odrive) == 0:
    print('Rebooting all odrive')
    for motor in odrives:
        input(f'Ready to reboot odrive of {motor}?')
        uart.send_reboot(odrives[motor])
        print('Done!')

        if debugging:
            time.sleep(1)
            while uart.ser.in_waiting > 0:
                data = uart.ser.read(uart.ser.in_waiting)
                print(" ".join(f"{b:02X}" for b in data))
else:
    input(f'Ready to reboot odrive of {motor}?')
    send_reboot(odrives[args.odrive])
    print('Done!')

    if debugging:
        time.sleep(1)
        while uart.ser.in_waiting > 0:
            data = uart.ser.read(uart.ser.in_waiting)
            print(" ".join(f"{b:02X}" for b in data))

time.sleep(1)
uart.close()
print('Uart closed')

