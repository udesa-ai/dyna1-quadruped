from classes.quadruped import Dyna
from classes.motors import Motors
import argparse

parser = argparse.ArgumentParser(description="Which odrive to reboot")

parser.add_argument('odrive', metavar='N', type=str, nargs='*')

args = parser.parse_args()

if len(args.odrive) == 0:
    print('Rebooting all odrive')
    perro = Dyna()
    perro.reboot_motors()
else:
    motores = Motors()
    motores.reboot_odrive(args.odrive)
