import numpy as np
import matplotlib.pyplot as plt

# Load the data from the file
file_path = 'calc_data.txt'  # change this to your actual file path
data = []

with open(file_path, 'r') as f:
    for line in f:
        # Convert each line into a list of floats
        numbers = eval(line.strip())  # safe here because it's controlled data
        if len(numbers) == 12:
            data.append(numbers)
        else:
            print(f"Skipping line with {len(numbers)} elements")

data = np.array(data)  # shape: (N, 12)
timesteps = np.arange(data.shape[0])

# Plot all 12 values over time
print(data[0, :])

