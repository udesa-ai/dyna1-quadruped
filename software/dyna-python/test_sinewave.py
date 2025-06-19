from classes.uart_bridge import UARTBridge
import time
import numpy as np
import matplotlib.pyplot as plt

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
    pos = uart.read_uart()
    center = pos['FRfoot']
    t0 = time.time()
    while True:
        time.sleep(1/T)
        setpoint = center + np.sin(2 * np.pi * 0.25 * (time.time() - t0)) * 0.2
        temp.append(setpoint)
        temp_time.append(time.time())

except KeyboardInterrupt:
       print('Exiting...')

uart.close()


# Plot the sine wave
plt.scatter(temp_time, temp)
plt.title('Sine Wave')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.grid()
plt.show()