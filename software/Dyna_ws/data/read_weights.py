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

day = ['20240326-143836',
       #'20240326-144640',
       '20240326-145438',
       '20240326-150138',
       '20240326-150801',
       '20240326-152019',
       '20240326-152805',
       '20240410-163125']

# Get latest data directory from data directory
current_working_directory = os.path.dirname(os.path.abspath(__file__))
list_of_files = glob.glob(os.path.join(current_working_directory,'*'))
list_of_files = [file for file in list_of_files if any(name in file for name in day)]
list_of_files.sort()

valores = {}

for file in list_of_files:
    # Read weight
    with open(os.path.join(file,'weight.txt'), 'r') as w:
        weight = int(w.readline())/1000
    
    valores[weight] = {'max_corriente_RMS_walk':0,
                       'max_potencia_RMS_walk':0,
                       'total_potencia_RMS_walk':0}

    # Read command csv
    command = pd.read_csv(os.path.join(file,'control.csv'))

    # Read angle measurement csv
    measured = pd.read_csv(os.path.join(file,'measured.csv'))

    # Read current measurements csv
    currents = pd.read_csv(os.path.join(file,'currents.csv'))

    allofthem = [f'{a}{b}' for a in ['fl', 'fr', 'bl', 'br'] for b in ['shoulder', 'arm', 'foot']]


    ############# RMS ##############


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

    n_times = len(timestamps)
    maxi = max([max(corriente) for corriente in dict_rms.values()])

    valores[weight]['max_corriente_RMS_walk'] = maxi    

    ############# POTENCIA INDIVIDUAL ##############

    Kt = 0.034408800303936005
    R = 0.0946

    i = 0
    leggo = 0

    dict_rms = {}

    tot_power = []

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
        if len(tot_power) == 0:
            tot_power = np.array(power)
        else:
            tot_power += np.array(power)

        for timeframe in timestamps:
            rms = 0
            n = 0
            for value, timi in zip(power, measured['time_start'][:-1]):
                if timeframe[0] < timi < timeframe[1]:
                    n += 1
                    rms += value**2
            true_rms = np.sqrt(rms/n)
            if leg in dict_rms:
                dict_rms[leg].append(true_rms)
            else:
                dict_rms[leg] = [true_rms]
        
        rms = 0
        n = 0

    for value, timi in zip(tot_power, measured['time_start'][:-1]):
        if timeframe[0] < timi < timeframe[1]:
            n += 1
            rms += value**2
    true_rms = np.sqrt(rms/n)

    n_times = len(timestamps)
    maxi = max([max(potencia) for potencia in dict_rms.values()])
    valores[weight]['max_potencia_RMS_walk'] = maxi
    valores[weight]['total_potencia_RMS_walk'] = true_rms


x = np.array([w for w in valores.keys()])
y = np.array([value['max_corriente_RMS_walk'] for value in valores.values()])

plt.figure(figsize = (5,4))
colores()
plt.grid()
plt.plot(x, y, label = 'Corriente RMS Max',  marker='o')

plt.xlabel('Peso agregado [kg]')
plt.ylabel('Corriente RMS máxima [A]')

peso = 4

z = np.polyfit(x, y, 2)
p = np.poly1d(z)
X = np.linspace(0,peso,100)
Y = p(X)
plt.plot(X, Y, '--', alpha = 0.5, label = 'Curva de Tendencia')

plt.legend()

plt.tight_layout()









x = [w for w in valores.keys()]
y1 = [value['max_potencia_RMS_walk'] for value in valores.values()]
y2 = [value['total_potencia_RMS_walk'] for value in valores.values()]


fig, ax1 = plt.subplots(figsize = (5, 4))
colores()


lns1 = ax1.plot(x, y1, label='Motor más exigido', marker='o')
ax1.set_yticks([20, 25, 30, 35, 40, 45, 50, 55, 60])
plt.grid()
ax2 = ax1.twinx()

lns2 = ax2.plot(x, y2, label='Total', marker='o', color = 'C1')


ax2.set_yticks(np.linspace(ax2.get_yticks()[0], ax2.get_yticks()[-1], len(ax1.get_yticks())))



ax1.set_xlabel('Peso agregado [kg]')
ax1.set_ylabel('Potencia RMS de motor max exigido [W]', color = 'C0')
ax2.set_ylabel('Potencia RMS total [W]', color = 'C1')

lns = lns1+lns2
labs = [l.get_label() for l in lns]
ax1.legend(lns, labs, loc=0)

fig.tight_layout()

############# SHOW ##############
plt.show()
