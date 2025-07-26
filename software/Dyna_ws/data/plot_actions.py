import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
import statistics

all_in_one = False
walking = False
get_this = ["brfoot"]

max_current = 50

# Get latest data directory from data directory
current_working_directory = os.path.dirname(os.path.abspath(__file__))
list_of_files = glob.glob(os.path.join(current_working_directory,'*'))
list_of_files = [file for file in list_of_files if '.py' not in file and 'csv' not in file and 'txt' not in file] 
list_of_files.sort()
# latest_file = max(list_of_files, key=os.path.getctime)

myfile = list_of_files[-1] # 4 6
print(myfile)

# Read command csv
command = pd.read_csv(os.path.join(myfile,'control.csv'))

# Read angle measurement csv
measured = pd.read_csv(os.path.join(myfile,'measured.csv'))

# Read current measurements csv
currents = pd.read_csv(os.path.join(myfile,'currents.csv'))

# Read imu measurements csv
imu = pd.read_csv(os.path.join(myfile,'imu.csv'))

# Check lengths
co = len(command)>1
me = len(measured)>1 
cu = len(currents)>1 

allofthem = [f'{a}{b}' for a in ['fl', 'fr', 'bl', 'br'] for b in ['shoulder', 'arm', 'foot']]


#############################################################################
######################## Plot measured and commanded ########################
#############################################################################
offset = [0, -0.79, 1.5]


plt.figure()
for index, axis in enumerate(allofthem):
    commanded = command[axis]/180*np.pi
    commanded -= offset[index%3]
    commanded *= 4
    plt.plot(command['time_start'], commanded, label = axis)
plt.legend()



############# SHOW ##############
plt.show(block=False)
plt.pause(0.001) # Pause for interval seconds.
input("hit[enter] to end.")
plt.close('all') # all open plots are correctly closed after each run