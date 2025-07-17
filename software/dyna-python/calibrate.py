from classes.motors_fake import Motors
from classes.uart_bridge import UARTBridge
import argparse
import json

parser = argparse.ArgumentParser(description="Which Leg to calibrate")

parser.add_argument('odrive', metavar='N', type=str, nargs='*')

args = parser.parse_args()

uart = UARTBridge()
uart.open()

motores = Motors()

if len(args.odrive) == 0:
    LEGS = ['FR', 'FL', 'BL', 'BR']
else:
    LEGS = args.odrive

print(f'Will calibrate {LEGS}')

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

for leg in LEGS:
    input(f'Ready to calibrate {leg}?')
    values = uart.read_uart()
    motores.set_motor_encoders(values)
    leggo = {f'{leg}shoulder': 0, f'{leg}arm':0, f'{leg}foot':0}
    motores.get_motor_positions(leggo)
    for index, mot in enumerate(leggo):
        c_angle = motores.data[leg][f'motor{index}']["calibration_angle"]
        pos0 = leggo[mot] - c_angle*motores.rrate/360
        motores.data[leg][f'motor{index}']["iOffset"] = pos0
        print(leggo[mot], c_angle, pos0)
        leggo[mot] = pos0
        
    motores.set_zero_positions(leggo)

with open("config.json",'w') as file:
    json.dump(motores.data, file, indent=4)

print('All motors calibrated and config file updated!')
values = uart.read_uart()
motores.set_motor_encoders(values)
print(motores.get_angle(pos))
