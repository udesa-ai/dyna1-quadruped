import csv
import matplotlib.pyplot as plt
import numpy as np

csv_filename = "data/log_step_curve_arm_30A.csv"

log_time = []
keys = ['FRshoulder', 'FRarm', 'FRfoot']
log_requested = {k: [] for k in keys}


time_measured = []
fr_shoulder = []
fr_arm = []
fr_foot = []

fr_shoulder_current = []
fr_arm_current = []
fr_foot_current = []

# --- Read CSV ---
with open(csv_filename, newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        # Requested
        if row["timestamp_request"]:
            log_time.append(float(row["timestamp_request"]))
            log_requested['FRshoulder'].append(float(row["FRshoulder_request"]))
            log_requested['FRarm'].append(float(row["FRarm_request"]))
            log_requested['FRfoot'].append(float(row["FRfoot_request"]))

        # Measured
        time_measured.append(float(row["timestamp_measured"]))
        fr_shoulder.append(float(row["FRshoulder_measured"]))
        fr_arm.append(float(row["FRarm_measured"]))
        fr_foot.append(float(row["FRfoot_measured"]))
        fr_shoulder_current.append(float(row["FRshoulder_current"]) if row["FRshoulder_measured"] else np.nan)
        fr_arm_current.append(float(row["FRarm_current"]))
        fr_foot_current.append(float(row["FRfoot_current"]))

time_step = log_time[-3]

log_time = np.array(log_time) - time_step
time_measured = np.array(time_measured) - time_step

plt.figure(figsize=(10, 8))
plt.subplot(3, 1, 1)
plt.plot(time_measured, fr_shoulder, label='Measured FRshoulder')
plt.title('FRshoulder')
plt.plot(log_time, log_requested['FRshoulder'], label='Requested FRshoulder')
plt.xlim(-1, max(max(log_time), max(time_measured)))
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time_measured, fr_arm, label='Measured FRarm')
plt.title('FRarm')
plt.plot(log_time, log_requested['FRarm'], label='Requested FRarm')
plt.xlim(-1, max(max(log_time), max(time_measured)))
plt.legend()

plt.subplot(3, 1, 3)

plt.plot(time_measured, fr_foot, label='Measured FRfoot')
plt.title('FRfoot')
plt.plot(log_time, log_requested['FRfoot'], label='Requested FRfoot')
plt.xlim(-1, max(max(log_time), max(time_measured)))
plt.legend()

plt.figure()
#plot difference between times
time_diff = np.diff(time_measured)
plt.plot(time_measured[1:], time_diff*1000)
plt.xlim(-1, max(max(log_time), max(time_measured)))
plt.title('Time difference between measurements')
plt.xlabel('Time (s)')
plt.ylabel('Time difference (s)')

plt.figure()
plt.subplot(3, 1, 1)
plt.plot(time_measured, fr_shoulder_current, label='FRshoulder Current')
plt.title('FRshoulder Current')
plt.xlim(-1, max(max(log_time), max(time_measured)))
plt.subplot(3, 1, 2)
plt.plot(time_measured, fr_arm_current, label='FRarm Current')
plt.xlim(-1, max(max(log_time), max(time_measured)))
plt.title('FRarm Current')
plt.subplot(3, 1, 3)
plt.plot(time_measured, fr_foot_current, label='FRfoot Current')
plt.xlim(-1, max(max(log_time), max(time_measured)))
plt.title('FRfoot Current')

plt.show()