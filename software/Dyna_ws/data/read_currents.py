import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
import statistics

get_this = "flarm"

max_current = 25

# Get latest data directory from data directory
current_working_directory = os.path.dirname(os.path.abspath(__file__))
list_of_files = glob.glob(os.path.join(current_working_directory,'*'))
list_of_files = [file for file in list_of_files if '.py' not in file and 'csv' not in file]
list_of_files.sort()
# latest_file = max(list_of_files, key=os.path.getctime)

myfile = list_of_files[-2] # 4 6
print(myfile)

# Read current measurements csv
currents = pd.read_csv(os.path.join(myfile,'currents.csv'))
time = currents['time_start'].to_numpy()
currents = currents[get_this].to_numpy()
# currents *= 50/30

#############################################################################
############################### Plot currents ###############################
#############################################################################

alpha1 = [0.01]
valores = {}

for al in alpha1:
    valores[al] = []

flag = True
for value in currents:
    if flag:
        flag = False
        for al in alpha1:
            valores[al].append(al*value)
    else:
        for al in alpha1:
            valores[al].append(al*value + (1-al)*valores[al][-1])


for al, values in valores.items():
    plt.plot(time,  values, label = al, color='orange')


plt.scatter(time, currents)

plt.legend()

plt.xlabel('Time [s]')
plt.ylabel('Current [A]')


plt.show()