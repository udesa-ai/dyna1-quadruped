
from classes.uart_bridge import UARTBridge

uart = UARTBridge()
uart.open()

uart.send_on_off('off')

print('Done!')
uart.close()
