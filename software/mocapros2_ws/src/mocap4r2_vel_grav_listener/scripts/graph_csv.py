import argparse
import csv
import glob
import os

import matplotlib.pyplot as plt
import numpy as np

DATA_DIR = os.path.join(os.path.dirname(__file__), '..', 'data')

VARIABLES = [
    'lin_vel_x', 'lin_vel_y', 'lin_vel_z',
    'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
    'lin_vel_fil_x', 'lin_vel_fil_y', 'lin_vel_fil_z',
    'ang_vel_fil_x', 'ang_vel_fil_y', 'ang_vel_fil_z',
    'gravity_x', 'gravity_y', 'gravity_z',
]


def latest_csv():
    files = glob.glob(os.path.join(DATA_DIR, 'vel_grav_*.csv'))
    if not files:
        raise FileNotFoundError(f'No vel_grav_*.csv files found in {DATA_DIR}')
    return max(files, key=os.path.getmtime)


def main():
    parser = argparse.ArgumentParser(description='Plot vel_grav_listener CSV data over time')
    parser.add_argument('csv_file', nargs='?', default=None,
                         help='Path to the CSV file (default: most recent in data/)')
    parser.add_argument('--output', type=str, default=None,
                         help='Output image path (default: same name as CSV, .png)')
    args = parser.parse_args()

    csv_path = args.csv_file if args.csv_file else latest_csv()

    time = []
    values = {var: [] for var in VARIABLES}

    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            time.append(float(row['timestamp']))
            for var in VARIABLES:
                values[var].append(float(row[var]))

    time = np.array(time)
    time -= time[0]

    fig, axes = plt.subplots(len(VARIABLES), 1, figsize=(10, 2.2 * len(VARIABLES)), sharex=True)
    for ax, var in zip(axes, VARIABLES):
        ax.plot(time, values[var])
        ax.set_ylabel(var)
        ax.grid(True)
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle(os.path.basename(csv_path))
    fig.tight_layout()

    output_path = args.output if args.output else os.path.splitext(csv_path)[0] + '.png'
    fig.savefig(output_path)
    print(f'Saved figure to: {output_path}')


if __name__ == '__main__':
    main()
