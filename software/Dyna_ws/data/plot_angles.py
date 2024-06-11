import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
import statistics

# Get latest data directory from data directory
current_working_directory = os.path.dirname(os.path.abspath(__file__))
list_of_files = glob.glob(os.path.join(current_working_directory,'*'))
list_of_files = [file for file in list_of_files if '.py' not in file and 'csv' not in file]
list_of_files.sort()
# latest_file = max(list_of_files, key=os.path.getctime)

myfile = list_of_files[-1] # 4 6

# Read angle measurement csv
measured = pd.read_csv(os.path.join(myfile,'measured.csv'))

allofthem = [f'{a}{b}' for a in ['fl', 'fr', 'bl', 'br'] for b in ['shoulder', 'arm', 'foot']]


#############################################################################
######################## Plot measured and commanded ########################
#############################################################################
for axiss in allofthem:
    plt.figure()
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [°]')
    plt.plot(measured['time_start'], measured[axiss])
    plt.title(axiss)
    plt.grid()


plt.show()