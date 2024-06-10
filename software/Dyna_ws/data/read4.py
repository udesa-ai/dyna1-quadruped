import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
import statistics

def colores():
    data = pd.read_csv('colores.csv')
    data = data.to_numpy()

    plt.rcParams["axes.prop_cycle"] = plt.cycler("color", data)
    return data

all_in_one = False
get_this = ['blshoulder', 'blarm', 'frfoot']

max_current = 50

# Get latest data directory from data directory
current_working_directory = os.path.dirname(os.path.abspath(__file__))
list_of_files = glob.glob(os.path.join(current_working_directory,'*'))
list_of_files = [file for file in list_of_files if '.py' not in file and '.csv' not in file]
list_of_files.sort()
# latest_file = max(list_of_files, key=os.path.getctime)

myfile = list_of_files[-1] # 4 6
print(myfile)


# Read command csv
command = pd.read_csv(os.path.join(myfile,'control.csv'))

command_time = np.array(command['time_start']) - command['time_start'].iloc[0]

# Read angle measurement csv
measured = pd.read_csv(os.path.join(myfile,'measured.csv'))

measured_time = np.array(measured['time_start']) - measured['time_start'].iloc[0]

# Read current measurements csv
currents = pd.read_csv(os.path.join(myfile,'currents.csv'))

currents_time = np.array(currents['time_start']) - currents['time_start'].iloc[0]

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
    colors = colores()
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [°]')
    if co:
        plt.scatter(command_time, command[axiss], alpha=0.5)
        # plt.plot(command_time, command[axiss], linestyle = ':', alpha = 0.5)
    if me:
        plt.plot(measured_time, measured[axiss], color = colors[1,:])
        plt.title(axiss)
        plt.grid()
    

#############################################################################
############################ Plot control jitter ############################
#############################################################################

if co or me or cu: 
    plt.figure(figsize=(6,6))
    data = colors = colores()
    plt.subplots_adjust(hspace=0.5)

cu = False

if co:
    freqs_c = np.array(command_time[1:])-np.array(command_time[:-1])
    if me and not cu:
        plt.subplot(2,2,1)
    elif me and cu:
        plt.subplot(3,2,1)
    elif cu and not me:
        plt.subplot(2,2,1)
    else:
        plt.subplot(1,2,1)
    plt.title(f'Delta de comando. $\sigma$ = {np.round(statistics.stdev(freqs_c*1000),3)}')
    plt.scatter(range(0,len(freqs_c)), freqs_c*1000, label = '$\Delta$T entre llamados', alpha=0.25)
    avg = sum(freqs_c)*1000/len(freqs_c)
    plt.plot([0, len(freqs_c)], [avg]*2, label = f'$\Delta$T promedio: {round(avg,3)} ms', color = data[1,:], linewidth = 2)
    plt.legend()
    plt.xlabel('N° de comando')
    plt.ylabel('Tiempo [ms]')
    plt.ylim([5, 20])
    if me and not cu:
        plt.subplot(2,2,2)
    elif me and cu:
        plt.subplot(3,2,2)
    elif cu and not me:
        plt.subplot(2,2,2)
    else:
        plt.subplot(1,2,2)
    plt.title('Histograma Comandos')
    # bins = 10**(np.linspace(np.log10(9),np.log10(14),20))
    # plt.xscale('log')  
    plt.hist(freqs_c*1000) #, bins = bins)
    plt.yscale('log')
    plt.xlim([7.5, 13])
    plt.xlabel('Período [ms]')
    plt.ylabel('Cantidad de instancias')
    

if me:
    freqs_m = np.array(measured_time[1:])-np.array(measured_time[:-1])
    if co and not cu:
        plt.subplot(2,2,3)
    elif co and cu:
        plt.subplot(3,2,3)
    elif cu and not co:
        plt.subplot(2,2,1)
    else:
        plt.subplot(1,2,1)
    plt.title(f'Delta de mediciones. $\sigma$ = {np.round(statistics.stdev(freqs_m*1000), 3)}')
    plt.scatter(range(0,len(freqs_m)), freqs_m*1000, label = '$\Delta$T entre llamados', alpha=0.25)
    avg = sum(freqs_m)*1000/len(freqs_m)
    plt.plot([0, len(freqs_m)], [avg]*2, label = f'$\Delta$T promedio: {round(avg,3)} ms', color = data[1,:], linewidth = 2)
    plt.legend()
    plt.xlabel('N° de medición')
    plt.ylabel('Tiempo [ms]')
    plt.ylim([5, 20])
    if co and not cu:
        plt.subplot(2,2,4)
    elif co and cu:
        plt.subplot(3,2,4)
    elif cu and not co:
        plt.subplot(2,2,2)
    else:
        plt.subplot(1,2,2)
    plt.title('Histograma Mediciones')
    # bins = 10**(np.linspace(np.log10(9),np.log10(14),20))
    # plt.xscale('log')
    plt.hist(freqs_m*1000) #, bins = bins)
    plt.yscale('log')
    plt.xlim([7.5, 13])
    plt.xlabel('Período [ms]')
    plt.ylabel('Cantidad de instancias')
    # plt.xticks(rotation = 60)

plt.tight_layout()

cu = True

#############################################################################
############################### Plot currents ###############################
#############################################################################
if cu:
    leggo = 0
    fig, axes = plt.subplots(2, 2,figsize=(15,9))
    axes = list(axes.flat)

    for name in allofthem:
        axes[leggo].plot(currents_time, currents[name], label = name[2:])
        pasado = [None if abs(curr) < 0.98*max_current else curr for curr in currents[name]]
        axes[leggo].scatter(currents_time, pasado, color = 'red')
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

if 'motion' in command:
    dict_rms = {}
    timestamps = []
    flag = False
    for timed, value, action in zip(command_time,command['motion'],command['movement']):
        if not flag and value == 'Go' and action == 'Stepping':
            flag = True
            timestamps.append([timed])
        elif flag and value != 'Go':
            flag = False
            timestamps[-1].append(timed)

    for current in allofthem:
        timeframe = timestamps[-1]
        rms = 0
        n = 0
        for value, timi in zip(currents[current], currents_time):
            if timeframe[0] < timi < timeframe[1]:
                n += 1
                rms += value**2
        true_rms = np.sqrt(rms/n)
        if current in dict_rms:
            dict_rms[current].append(true_rms)
        else:
            dict_rms[current] = [true_rms]

    plt.figure(figsize = (6,3))
    plt.rc('axes', axisbelow=True)
    plt.grid()
    colores()
    n_times = 1
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
    plt.ylim((0,16))
    plt.tight_layout()


############# POTENCIA INDIVIDUAL ##############

Kt = 0.034408800303936005
R = 0.0946

powers = {}
i = 0
leggo = 0

for leg in allofthem:
    torque = currents[leg].to_numpy() * Kt
    delta = (measured[leg].to_numpy()[1:] - measured[leg].to_numpy()[:-1])*9/360
    delta_time = measured_time[1:] - measured_time[:-1]
    vel = delta/(delta_time)

    mechanical_power = torque[:-1] * vel

    electrical_power = currents[leg].to_numpy() * currents[leg].to_numpy() * R
    delta_time_current = currents[leg].to_numpy()[1:] - currents[leg].to_numpy()[:-1]

    for index, value in enumerate(mechanical_power):
        if value < 0:
            mechanical_power[index] = 0

    power = mechanical_power + electrical_power[:-1]

    powers[leg] = {'mech':mechanical_power, 'ele':electrical_power[:-1]}

    i += 1
    
    
    if i == 3:
        i = 0
        leggo += 1

############# POTENCIA TOTAL ##############

plt.figure()
colores()


values = {}

for leg in allofthem:
    if 'mech' not in values:
        values['mech'] = powers[leg]['mech']
        values['ele'] = powers[leg]['ele']
    else:
        values['mech'] += powers[leg]['mech']
        values['ele'] += powers[leg]['ele']

plt.plot(measured_time[:-1], values['mech'],  label = 'Mechanical Power')
plt.plot(measured_time[:-1], values['ele'], label = 'Joule Heating')
#plt.plot(measured_time[:-1], values['mech'] + values['ele'], ":", label = 'Power', alpha = 0.5)
plt.legend()
plt.title('Total Power')
plt.xlabel('Time [S]')
plt.ylabel('Power [W]')
plt.grid()





############# SHOW ##############
plt.show(block=False)
plt.pause(0.001) # Pause for interval seconds.
input("hit[enter] to end.")
plt.close('all') # all open plots are correctly closed after each run