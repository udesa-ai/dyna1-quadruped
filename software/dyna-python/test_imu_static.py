from classes.uart_bridge import UARTBridge
import time
import argparse
import csv


def interpolate_timestamps(timestamps):
    """Spread out frames that share the same host read timestamp (because
    several UART frames arrived in the same read_all() burst) evenly across
    the gap to the next distinct timestamp, so consecutive samples get
    monotonically increasing times instead of duplicates."""
    n = len(timestamps)
    result = list(timestamps)
    last_step = 0.0
    i = 0
    while i < n:
        j = i
        while j + 1 < n and timestamps[j + 1] == timestamps[i]:
            j += 1
        group_size = j - i + 1

        if j + 1 < n:
            step = (timestamps[j + 1] - timestamps[i]) / group_size
            last_step = step
        else:
            step = last_step

        for k in range(group_size):
            result[i + k] = timestamps[i] + k * step

        i = j + 1

    return result


# arpgarse to pass duration of the measurement
parser = argparse.ArgumentParser(description="Log IMU data with the robot standing still")
parser.add_argument('--duration', type=float, default=100.0, help='Duration of the measurement in seconds')
parser.add_argument('--output', type=str, default='software/dyna-python/data/log_imu_static_100.txt', help='Raw log output file')
parser.add_argument('--csv', type=str, default='software/dyna-python/data/log_imu_static_100.csv', help='CSV output file')
args = parser.parse_args()

# Initialize variables
uart = UARTBridge()
uart.open()

T = args.duration

input(f'Press Enter to start logging IMU data for {T} seconds (robot should remain still)...')

t0 = time.monotonic()

uart.start_logging()

try:
    while time.monotonic() - t0 < T:
        time.sleep(0.01)
except KeyboardInterrupt:
    print('Exiting...')

uart.stop_logging()
uart.save_log(args.output)
uart.close()

parsed_data = uart.parser(args.output)

timestamps = interpolate_timestamps([entry['timestamp'] for entry in parsed_data])

csv_filename = args.csv
with open(csv_filename, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([
        "timestamp",
        "accel_x", "accel_y", "accel_z",
        "gyro_x", "gyro_y", "gyro_z"
    ])
    for entry, ts in zip(parsed_data, timestamps):
        accel = entry['imu']['accel']
        gyro = entry['imu']['gyro']
        writer.writerow([
            ts - t0,
            accel[0], accel[1], accel[2],
            gyro[0], gyro[1], gyro[2]
        ])

print(f"CSV saved to {csv_filename}")
