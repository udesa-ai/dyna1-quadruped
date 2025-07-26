import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
import numpy as np
from ahrs.filters import Madgwick
from ahrs.common.orientation import q2R

# Get latest data directory from data directory
current_working_directory = os.path.dirname(os.path.abspath(__file__))
list_of_files = glob.glob(os.path.join(current_working_directory,'*'))
list_of_files = [file for file in list_of_files if '.py' not in file and 'csv' not in file and 'txt' not in file] 
list_of_files.sort()
# latest_file = max(list_of_files, key=os.path.getctime)

myfile = list_of_files[-1] # 4 6
print(myfile)

# Read imu measurements csv
imu = pd.read_csv(os.path.join(myfile,'imu.csv'))

############# IMU ##############
imu_x = -imu['la_z'].values
imu_y = -imu['la_x'].values
imu_z = -imu['la_y'].values

# Normalize the accelerometer data
acc_norm = np.sqrt(imu_x**2 + imu_y**2 + imu_z**2)
imu_x /= acc_norm
imu_y /= acc_norm
imu_z /= acc_norm


#time_start,av_x,av_y,av_z,la_x,la_y,la_z
plt.figure()
plt.plot(imu_x, label = 'linear acceleration x')
plt.plot(imu_y, label = 'linear acceleration y')
plt.plot(imu_z, label = 'linear acceleration z')
plt.legend()

plt.figure()
plt.plot(imu['av_x'], label = 'angular velocity x')
plt.plot(imu['av_y'], label = 'angular velocity y')
plt.plot(imu['av_z'], label = 'angular velocity z')
plt.legend()


# Initialize filter
madgwick = Madgwick()

# Convert data to numpy arrays
acc = imu[['la_z', 'la_x', 'la_y']].values
gyr = imu[['av_z', 'av_x', 'av_y']].values/180*np.pi

quaternions = np.zeros((len(acc), 4))
quaternions[0] = [np.sqrt(2), 0, 0, np.sqrt(2)]  # Initial orientation

for t in range(1, len(acc)):
    quaternions[t] = madgwick.updateIMU(quaternions[t-1], gyr[t], acc[t])

# Project gravity at time t
t = 100  # choose time index
R = q2R(quaternions[t])  # Rotation matrix from world to body
# Compute g_body for each time step
g_world = np.array([0, 0, -1])
g_body = np.zeros((len(acc), 3))
g_norm_squared = np.zeros(len(acc))

for t in range(len(acc)):
    R = q2R(quaternions[t])
    g_body[t] = R @ g_world  # gravity in sensor frame
    g_norm_squared[t] = np.sum(g_body[t]**2)  # squared magnitude

time = imu['time_start']
# Plot
plt.figure(figsize=(10, 5))
plt.plot(g_body[:, 0], label='g_x (body)')
plt.plot(g_body[:, 1], label='g_y (body)')
plt.plot(g_body[:, 2], label='g_z (body)')
plt.plot(g_norm_squared, color='black', label='|g_body|²')
plt.title("Projected Gravity in Body Frame")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s²]")
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()


############# SHOW ##############
plt.show(block=False)
plt.pause(0.001) # Pause for interval seconds.
input("hit[enter] to end.")
plt.close('all') # all open plots are correctly closed after each run