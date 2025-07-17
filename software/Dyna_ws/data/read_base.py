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
list_of_files = [file for file in list_of_files if '.py' not in file and 'csv' not in file]
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
if all_in_one:
    get_this = allofthem

for axiss in get_this:
    plt.figure()
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [°]')
    if co:
        plt.scatter(command['time_start'], command[axiss], alpha=0.5, color='orange')
        # plt.plot(command['time_start'], command[axiss], linestyle = ':', alpha = 0.5)
    if me:
        plt.plot(measured['time_start'], measured[axiss])
        plt.title(axiss)
        plt.grid()


plt.figure()
for axis in allofthem:
    plt.plot(command['time_start'], command[axis], label = axis)
plt.legend()

#############################################################################
############################ Plot control jitter ############################
#############################################################################

if co or me or cu: 
    plt.figure(figsize=(15,9))
    plt.subplots_adjust(hspace=0.5)

if co:
    freqs_c = np.array(command['time_start'][1:])-np.array(command['time_start'][:-1])
    if me and not cu:
        plt.subplot(2,2,1)
    elif me and cu:
        plt.subplot(3,2,1)
    elif cu and not me:
        plt.subplot(2,2,1)
    else:
        plt.subplot(1,2,1)
    plt.title(f'Command Delta. $\sigma$ = {statistics.stdev(freqs_c*1000)}')
    plt.scatter(range(0,len(freqs_c)), freqs_c*1000, label = '$\Delta$T between calls', alpha=0.25)
    avg = sum(freqs_c)*1000/len(freqs_c)
    plt.plot([0, len(freqs_c)], [avg]*2, label = f'Average $\Delta$T: {round(avg,3)} ms', color = 'orange')
    plt.legend()
    plt.xlabel('N° of command')
    plt.ylabel('time [ms]')
    plt.ylim([0, 30])
    if me and not cu:
        plt.subplot(2,2,2)
    elif me and cu:
        plt.subplot(3,2,2)
    elif cu and not me:
        plt.subplot(2,2,2)
    else:
        plt.subplot(1,2,2)
    plt.title('Command Hist')
    bins = 10**(np.linspace(0,2.3,100))
    plt.xscale('log')  
    plt.hist(freqs_c*1000, bins = bins)
    plt.yscale('log')

if me:
    freqs_m = np.array(measured['time_start'][1:])-np.array(measured['time_start'][:-1])
    if co and not cu:
        plt.subplot(2,2,3)
    elif co and cu:
        plt.subplot(3,2,3)
    elif cu and not co:
        plt.subplot(2,2,1)
    else:
        plt.subplot(1,2,1)
    plt.title(f'Measured Delta. $\sigma$ = {statistics.stdev(freqs_m*1000)}')
    plt.scatter(range(0,len(freqs_m)), freqs_m*1000, label = '$\Delta$T between calls', alpha=0.25)
    avg = sum(freqs_m)*1000/len(freqs_m)
    plt.plot([0, len(freqs_m)], [avg]*2, label = f'Average $\Delta$T: {round(avg,3)} ms', color = 'orange')
    plt.legend()
    plt.xlabel('N° of Measure')
    plt.ylabel('time [ms]')
    plt.ylim([0, 30])
    if co and not cu:
        plt.subplot(2,2,4)
    elif co and cu:
        plt.subplot(3,2,4)
    elif cu and not co:
        plt.subplot(2,2,2)
    else:
        plt.subplot(1,2,2)
    plt.title('Measured Hist')
    bins = 10**(np.linspace(0,2.3,100))
    plt.xscale('log')  
    plt.hist(freqs_m*1000, bins = bins)
    plt.yscale('log')

if cu:
    freqs_cu = np.array(currents['time_start'][1:])-np.array(currents['time_start'][:-1])
    if co and not me:
        plt.subplot(2,2,3)
    elif co and me:
        plt.subplot(3,2,5)
    elif me and not co:
        plt.subplot(2,2,3)
    else:
        plt.subplot(1,2,1)
        
    plt.title(f'Current Delta. $\sigma$ = {statistics.stdev(freqs_cu*1000)}')
    plt.scatter(range(0,len(freqs_cu)), freqs_cu*1000, label = '$\Delta$T between calls', alpha=0.25)
    avg = sum(freqs_cu)*1000/len(freqs_cu)
    plt.plot([0, len(freqs_cu)], [avg]*2, label = f'Average $\Delta$T: {round(avg,3)} ms', color = 'orange')
    plt.legend()
    plt.xlabel('N° of Current')
    plt.ylabel('time [ms]')
    plt.ylim([0, 30])
    if co and not me:
        plt.subplot(2,2,4)
    elif co and me:
        plt.subplot(3,2,6)
    elif me and not co:
        plt.subplot(2,2,4)
    else:
        plt.subplot(1,2,2)
    plt.title('Current Hist')
    bins = 10**(np.linspace(0,2.3,100))
    plt.xscale('log')  
    plt.hist(freqs_cu*1000, bins = bins)
    plt.yscale('log')


#############################################################################
############################### Plot currents ###############################
#############################################################################
if cu:
    leggo = 0
    fig, axes = plt.subplots(2, 2,figsize=(15,9))
    axes = list(axes.flat)

    for name in allofthem:
        axes[leggo].scatter(currents['time_start'], currents[name], label = name[2:])
        pasado = [None if abs(curr) < 0.98*max_current else curr for curr in currents[name]]
        axes[leggo].scatter(currents['time_start'], pasado, color = 'red')
        if 'foot' in name:
            if leggo == 0:
                texto = 'Front Left'
            elif leggo == 1:
                texto = 'Front Right'
            elif leggo == 2:
                texto = 'Back Left'
            elif leggo == 3:
                texto = 'Back Right'
            axes[leggo].title.set_text(texto)
            axes[leggo].legend()
            axes[leggo].grid()
            leggo += 1

    fig.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor='none', which='both', top=False, bottom=False, left=False, right=False)
    plt.xlabel('Time [s]')
    plt.ylabel('Current [A]')


############# RMS ##############

if 'motion' in command and walking:
    dict_rms = {}
    timestamps = []
    flag = False
    for timed, value, action in zip(command['time_start'],command['motion'],command['movement']):
        if not flag and value == 'Go' and action == 'Stepping':
            flag = True
            timestamps.append([timed])
        elif flag and value != 'Go':
            flag = False
            timestamps[-1].append(timed)

    for current in allofthem:
        for timeframe in timestamps:
            rms = 0
            n = 0
            for value, timi in zip(currents[current], currents['time_start']):
                if timeframe[0] < timi < timeframe[1]:
                    n += 1
                    rms += value**2
            true_rms = np.sqrt(rms/n)
            if current in dict_rms:
                dict_rms[current].append(true_rms)
            else:
                dict_rms[current] = [true_rms]

    plt.figure()
    n_times = len(timestamps)
    alpha = 0.7/n_times
    for i in range(n_times):
        x = [beta -(alpha*(n_times/2 - 0.5)) + i * alpha for beta in range(0,12)]
        y = [value[i] for value in dict_rms.values()]
        plt.bar(x, y, width = alpha, align='center')


    # plt.bar(dict_rms.keys(), dict_rms.values())
    plt.xticks(range(0,12), dict_rms.keys(), rotation = 90)
    plt.title('Corriente RMS mientras camina')
    plt.xlabel('Motor')
    plt.ylabel('Corriente [A]')
    plt.tight_layout()

############# POTENCIA INDIVIDUAL ##############

Kt = 0.034408800303936005
R = 0.0946

powers = {}
i = 0
leggo = 0

fig, axes = plt.subplots(2, 2,figsize=(15,9))
axes = list(axes.flat)

for leg in allofthem:
    torque = currents[leg].to_numpy() * Kt
    delta = (measured[leg].to_numpy()[1:] - measured[leg].to_numpy()[:-1])*9/360
    delta_time = measured['time_start'].to_numpy()[1:] - measured['time_start'].to_numpy()[:-1]
    vel = delta/(delta_time)

    mechanical_power = torque[:-1] * vel

    electrical_power = currents[leg].to_numpy() * currents[leg].to_numpy() * R
    delta_time_current = currents[leg].to_numpy()[1:] - currents[leg].to_numpy()[:-1]

    for index, value in enumerate(mechanical_power):
        if value < 0:
            mechanical_power[index] = 0

    power = mechanical_power + electrical_power[:-1]

    powers[leg] = {'mech':mechanical_power, 'ele':electrical_power[:-1]}

    if leggo == 0:
        axes[leggo].title.set_text('Front Left')
    elif leggo == 1:
        axes[leggo].title.set_text('Front Rigth')
    elif leggo == 2:
        axes[leggo].title.set_text('Back Left')
    elif leggo == 3:
        axes[leggo].title.set_text('Back Right')

    axes[leggo].plot(measured['time_start'][:-1], power, label = leg)
    i += 1
    
    
    if i == 3:
        axes[leggo].legend()
        axes[leggo].grid()
        i = 0
        leggo += 1
        

    # energy = np.sum(np.sum(abs(mechanical_power)) * delta_time) + \
    #          np.sum(np.sum(electrical_power[:-1]) * delta_time_current)
    
    # eps_forwards = energy/(measured['time_start'][-1] - measured['time_start'][0])  # + 15 + 0.15*12

    # eps_forwards = (energy/(0.8 * 0.925))/(measured['time_start'][-1] - measured['time_start'][0])  # + 15 + 0.15*12

fig.add_subplot(111, frameon=False)
plt.tick_params(labelcolor='none', which='both', top=False, bottom=False, left=False, right=False)
plt.xlabel('Time [s]')
plt.ylabel('Potencia [W]')

############# POTENCIA TOTAL ##############

plt.figure()
values = {}

for leg in allofthem:
    if 'mech' not in values:
        values['mech'] = powers[leg]['mech']
        values['ele'] = powers[leg]['ele']
    else:
        values['mech'] += powers[leg]['mech']
        values['ele'] += powers[leg]['ele']

plt.plot(measured['time_start'][:-1], values['mech'],  label = 'Mechanical Power')
plt.plot(measured['time_start'][:-1], values['ele'], label = 'Joule Heating')
#plt.plot(measured['time_start'][:-1], values['mech'] + values['ele'], ":", label = 'Power', alpha = 0.5)
plt.legend()
plt.title('Total Power')
plt.xlabel('Time [S]')
plt.ylabel('Power [W]')
plt.grid()

############# IMU ##############

#time_start,av_x,av_y,av_z,la_x,la_y,la_z

plt.figure()
plt.plot(imu['la_x'], label = 'linear acceleration x')
plt.plot(imu['la_y'], label = 'linear acceleration y')
plt.plot(imu['la_z'], label = 'linear acceleration z')
plt.legend()

plt.figure()
plt.plot(imu['av_x'], label = 'angular velocity x')
plt.plot(imu['av_y'], label = 'angular velocity y')
plt.plot(imu['av_z'], label = 'angular velocity z')
plt.legend()

############# SHOW ##############
plt.show(block=False)
plt.pause(0.001) # Pause for interval seconds.
input("hit[enter] to end.")
plt.close('all') # all open plots are correctly closed after each run