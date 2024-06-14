from classes.quadruped import Dyna
import argparse

parser = argparse.ArgumentParser(description="Which Leg to calibrate")

parser.add_argument('odrive', metavar='N', type=str, nargs='*')

args = parser.parse_args()


perro = Dyna()
# perro.reboot_motors()
# time.sleep(1)

if len(args.odrive) == 0:
    perro.calibrate_motors()
else:
    perro.calibrate_motors(args.odrive)
print(perro.get_angles())
print(perro.get_xyz())
