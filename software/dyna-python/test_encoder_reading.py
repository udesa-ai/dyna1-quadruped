from classes.motors import Motors
import time

def print_dict(my_dict):
    starts = ['FR', 'FL', 'BR', 'BL']
    for i in range(4):
        for key, value in my_dict.items():
            if starts[i] in key:
                print(f'{key}: {value}',end='   ')
        print()


motores = Motors()

pos = dict.fromkeys(motores.names,None)

print('############################################################')

input('ready?')

motores.get_motor_positions(pos)
print_dict(pos)
motores.get_angle(pos)
print()
print_dict(pos)

