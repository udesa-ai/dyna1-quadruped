from classes.uart_bridge import UARTBridge
import time
import numpy as np
import argparse
from classes.motors_fake import Motors
import matplotlib.pyplot as plt
import csv

# arpgarse to pass period and amplitude
parser = argparse.ArgumentParser(description="Step curve tester for motors")
parser.add_argument('--choice', type=str, default='None', help='Extremity to test: FRarm, FRshoulder, FRfoot')
args = parser.parse_args()

# Initialize variables
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

starting_pos = {'FRshoulder':0,
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

desired_pos = {'FRshoulder':0,
       'FRarm':0,
       'FRfoot':90,
       'FLshoulder':0,
       'FLarm':0,
       'FLfoot':90,
       'BLshoulder':0,
       'BLarm':0,
       'BLfoot':90,
       'BRshoulder':0,
       'BRarm':0,
       'BRfoot':90}

desired_pos_step = {'FRshoulder':20,
       'FRarm':45,
       'FRfoot':45,
       'FLshoulder':0,
       'FLarm':0,
       'FLfoot':90,
       'BLshoulder':0,
       'BLarm':0,
       'BLfoot':90,
       'BRshoulder':0,
       'BRarm':0,
       'BRfoot':90}

request = {'FRshoulder':0,
       'FRarm':0,
       'FRfoot':90,
       'FLshoulder':0,
       'FLarm':0,
       'FLfoot':90,
       'BLshoulder':0,
       'BLarm':0,
       'BLfoot':90,
       'BRshoulder':0,
       'BRarm':0,
       'BRfoot':90}

choice = args.choice  # Extremity to test

# Data containers
keys = ['FRshoulder', 'FRarm', 'FRfoot']
log_time = []
log_requested = {k: [] for k in keys}


input('Press Enter to turn on Front Right motors...')
uart.send_on_off(state='on', motor='FRshoulder')
uart.send_on_off(state='on', motor='FRarm')
uart.send_on_off(state='on', motor='FRfoot')

input('Press Enter to go to default position...')
values = uart.read_uart()
motores.set_motor_encoders(values)
motores.get_angle(pos)
motores.get_angle(starting_pos)
diference = {key: desired_pos[key] - pos[key] for key in pos.keys()}

T = 2
dt = 0.01

t0 = time.monotonic()
t1 = time.monotonic()

uart.start_logging()

try:
    while time.monotonic() - t0 < T:
        if time.monotonic() - t1 > dt:
            t1 = time.monotonic()
            log_time.append(time.monotonic()-t0)
            # values = uart.read_uart()
            # motores.set_motor_encoders(values)
            # motores.get_angle(pos)

            for key in request.keys():
                request[key] = starting_pos[key] + diference[key] * (time.monotonic() - t0) / T
        
            for key in keys:
                log_requested[key].append(request[key])

            motores.angle_to_position(request)
            uart.send_positions(request)

except KeyboardInterrupt:
    uart.send_on_off(state='off', motor='FRshoulder')
    uart.send_on_off(state='off', motor='FRarm')
    uart.send_on_off(state='off', motor='FRfoot')
    print('Exiting...')

input("Press Enter to perform step curve...")

try:
    for key in request.keys():
        if key == choice:
            request[key] = desired_pos_step[key]
        else:
            request[key] = desired_pos[key]
    t_step = time.monotonic()
    motores.angle_to_position(request)
    uart.send_positions(request)

except KeyboardInterrupt:
    uart.send_on_off(state='off', motor='FRshoulder')
    uart.send_on_off(state='off', motor='FRarm')
    uart.send_on_off(state='off', motor='FRfoot')
    print('Exiting...')

input('Press Enter to turn off Front Right motors...')
uart.send_on_off(state='off', motor='FRshoulder')
uart.send_on_off(state='off', motor='FRarm')
uart.send_on_off(state='off', motor='FRfoot')
uart.stop_logging()
uart.save_log('data/log_step_curve.txt')

parsed_data = uart.parser('data/log_step_curve.txt')

time_measured = [entry['timestamp']-t0 for entry in parsed_data]
fr_shoulder = [entry['motors']['FRshoulder']['pos'] for entry in parsed_data]
fr_arm = [entry['motors']['FRarm']['pos'] for entry in parsed_data]
fr_foot = [entry['motors']['FRfoot']['pos'] for entry in parsed_data]

fr_shoulder_current = [entry['motors']['FRshoulder']['curr'] for entry in parsed_data]
fr_arm_current = [entry['motors']['FRarm']['curr'] for entry in parsed_data]
fr_foot_current = [entry['motors']['FRfoot']['curr'] for entry in parsed_data]

for i in range(len(fr_shoulder)):
    values['FRshoulder']['pos'] = fr_shoulder[i]
    values['FRarm']['pos'] = fr_arm[i]
    values['FRfoot']['pos'] = fr_foot[i]
    motores.set_motor_encoders(values)
    motores.get_angle(pos)
    fr_shoulder[i] = pos['FRshoulder']
    fr_arm[i] = pos['FRarm']
    fr_foot[i] = pos['FRfoot']

log_time.append(t_step - t0)
log_time.append(t_step - t0)
log_time.append(time_measured[-1])

plt.figure()
plt.subplot(3, 1, 1)
plt.plot(time_measured, fr_shoulder, label='Measured FRshoulder')
plt.title('FRshoulder')
# also add deisred position line
plt.axhline(y=desired_pos['FRshoulder'], color='r', linestyle='--', label='Desired FRshoulder')
if choice == 'FRshoulder':
    plt.axhline(y=desired_pos_step['FRshoulder'], color='g', linestyle='--', label='Step FRshoulder')
    log_requested['FRshoulder'].append(desired_pos['FRshoulder'])
    log_requested['FRshoulder'].append(desired_pos_step['FRshoulder'])
    log_requested['FRshoulder'].append(desired_pos_step['FRshoulder'])
else:
    log_requested['FRshoulder'].append(desired_pos['FRshoulder'])
    log_requested['FRshoulder'].append(desired_pos['FRshoulder'])
    log_requested['FRshoulder'].append(desired_pos['FRshoulder'])
plt.plot(log_time, log_requested['FRshoulder'], label='Requested FRshoulder')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time_measured, fr_arm, label='Measured FRarm')
plt.title('FRarm')
# also add deisred position line
plt.axhline(y=desired_pos['FRarm'], color='r', linestyle='--', label='Desired FRarm')
if choice == 'FRarm':
    plt.axhline(y=desired_pos_step['FRarm'], color='g', linestyle='--', label='Step FRarm')
    log_requested['FRarm'].append(desired_pos['FRarm'])
    log_requested['FRarm'].append(desired_pos_step['FRarm'])
    log_requested['FRarm'].append(desired_pos_step['FRarm'])
else:
    log_requested['FRarm'].append(desired_pos['FRarm'])
    log_requested['FRarm'].append(desired_pos['FRarm'])
    log_requested['FRarm'].append(desired_pos['FRarm'])
plt.plot(log_time, log_requested['FRarm'], label='Requested FRarm')
plt.legend()

plt.subplot(3, 1, 3)

plt.plot(time_measured, fr_foot, label='Measured FRfoot')
plt.title('FRfoot')
plt.axhline(y=desired_pos['FRfoot'], color='r', linestyle='--', label='Desired FRfoot')
if choice == 'FRfoot':
    plt.axhline(y=desired_pos_step['FRfoot'], color='g', linestyle='--', label='Step FRfoot')
    log_requested['FRfoot'].append(desired_pos['FRfoot'])
    log_requested['FRfoot'].append(desired_pos_step['FRfoot'])
    log_requested['FRfoot'].append(desired_pos_step['FRfoot'])
else:
    log_requested['FRfoot'].append(desired_pos['FRfoot'])
    log_requested['FRfoot'].append(desired_pos['FRfoot'])
    log_requested['FRfoot'].append(desired_pos['FRfoot'])

plt.plot(log_time, log_requested['FRfoot'], label='Requested FRfoot')
plt.legend()

plt.figure()
#plot difference between times
time_diff = np.diff(time_measured)
plt.plot(time_measured[1:], time_diff*1000)
plt.title('Time difference between measurements')
plt.xlabel('Time (s)')
plt.ylabel('Time difference (s)')
plt.ylim(0, 30)

plt.figure()
plt.subplot(3, 1, 1)
plt.plot(time_measured, fr_shoulder_current, label='FRshoulder Current')
plt.title('FRshoulder Current')
plt.subplot(3, 1, 2)
plt.plot(time_measured, fr_arm_current, label='FRarm Current')
plt.title('FRarm Current')
plt.subplot(3, 1, 3)
plt.plot(time_measured, fr_foot_current, label='FRfoot Current')
plt.title('FRfoot Current')

plt.show()

csv_filename = f'data/log_step_curve_{choice}.csv'
max_len = max(len(log_time), len(time_measured))
with open(csv_filename, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([
        "timestamp_request", "FRshoulder_request", "FRarm_request", "FRfoot_request",
        "timestamp_measured", "FRshoulder_measured", "FRarm_measured", "FRfoot_measured",
        "FRshoulder_current", "FRarm_current", "FRfoot_current"
    ])
    for i in range(max_len):
        row = [
            log_time[i] if i < len(log_time) else '',
            log_requested['FRshoulder'][i] if i < len(log_requested['FRshoulder']) else '',
            log_requested['FRarm'][i] if i < len(log_requested['FRarm']) else '',
            log_requested['FRfoot'][i] if i < len(log_requested['FRfoot']) else '',
            time_measured[i] if i < len(time_measured) else '',
            fr_shoulder[i] if i < len(fr_shoulder) else '',
            fr_arm[i] if i < len(fr_arm) else '',
            fr_foot[i] if i < len(fr_foot) else '',
            fr_shoulder_current[i] if i < len(fr_shoulder_current) else '',
            fr_arm_current[i] if i < len(fr_arm_current) else '',
            fr_foot_current[i] if i < len(fr_foot_current) else ''
        ]
        writer.writerow(row)

    print(f"CSV saved to {csv_filename}")

# # while loop until user interrupts with Ctrl+C
# try:
#     pos = uart.read_uart()
#     center0 = pos['FRshoulder']['pos']
#     center1 = pos['FRarm']['pos']
#     center2 = pos['FRfoot']['pos']
#     pos = {'FRshoulder':center0, 'FRarm':center1, 'FRfoot':center2, 'FLshoulder':0, 'FLarm':0, 'FLfoot':0, 'BLshoulder':0, 'BLarm':0, 'BLfoot':0, 'BRshoulder':0, 'BRarm':0, 'BRfoot':0 }
#     t0 = time.monotonic()
#     while True:
#         time.sleep(1/T)
#         setpoint = np.sin(2 * np.pi * frequency * (time.monotonic() - t0)) * amplitude
#         pos['FRshoulder'] = setpoint + center0
#         pos['FRarm'] = setpoint + center1
#         pos['FRfoot'] = setpoint + center2
#         uart.send_positions(pos)

# except KeyboardInterrupt:
#     uart.send_on_off('off')
#     print('Exiting...')

uart.close()
