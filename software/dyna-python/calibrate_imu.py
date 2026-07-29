from classes.uart_bridge import UARTBridge
import time
import argparse
import os


YAML_PATHS = [
    os.path.join(os.path.dirname(__file__), '..', 'Dyna_ws', 'src', 'controler_cpp', 'config', 'imu_params.yaml'),
    os.path.join(os.path.dirname(__file__), '..', 'Dyna_ws', 'install', 'controler_cpp', 'share', 'controler_cpp', 'config', 'imu_params.yaml'),
]


def update_yaml_offsets(path, gyro_offset):
    if not os.path.isfile(path):
        return False

    with open(path, 'r') as f:
        lines = f.readlines()

    keys = {
        'gyro_offset_x': gyro_offset[0],
        'gyro_offset_y': gyro_offset[1],
        'gyro_offset_z': gyro_offset[2],
    }

    for i, line in enumerate(lines):
        stripped = line.strip()
        for key, value in keys.items():
            if stripped.startswith(f'{key}:'):
                indent = line[:len(line) - len(line.lstrip())]
                comment = ''
                if '#' in line:
                    comment = ' ' + line[line.index('#'):].rstrip('\n')
                lines[i] = f'{indent}{key}: {value:.10g}{comment}\n'

    with open(path, 'w') as f:
        f.writelines(lines)

    return True


def main():
    parser = argparse.ArgumentParser(description="Calibrate IMU gyro bias with the robot standing still")
    parser.add_argument('--duration', type=float, default=10.0, help='Duration of the measurement in seconds')
    parser.add_argument('--raw-log', type=str, default='data/calib_imu_raw.txt', help='Raw UART log output file')
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.raw_log) or '.', exist_ok=True)

    uart = UARTBridge()
    uart.open()

    input(f'Press Enter to start calibrating the gyro for {args.duration} seconds (robot must remain still)...')

    uart.start_logging()
    try:
        time.sleep(args.duration)
    except KeyboardInterrupt:
        print('Interrupted, using data collected so far...')
    uart.stop_logging()
    uart.save_log(args.raw_log)
    uart.close()

    parsed_data = uart.parser(args.raw_log)
    if not parsed_data:
        print('No IMU data captured, aborting.')
        return

    n = len(parsed_data)
    sums = [0.0, 0.0, 0.0]
    for entry in parsed_data:
        gyro = entry['imu']['gyro']
        for i in range(3):
            sums[i] += gyro[i]

    gyro_offset = [s / n for s in sums]

    print(f'Samples: {n}')
    print(f'Gyro offset (deg/s): x={gyro_offset[0]:.6f}, y={gyro_offset[1]:.6f}, z={gyro_offset[2]:.6f}')

    updated_any = False
    for path in YAML_PATHS:
        path = os.path.normpath(path)
        if update_yaml_offsets(path, gyro_offset):
            print(f'Updated {path}')
            updated_any = True
        else:
            print(f'Skipped (not found): {path}')

    if not updated_any:
        print('Warning: no imu_params.yaml found to update!')


if __name__ == '__main__':
    main()
