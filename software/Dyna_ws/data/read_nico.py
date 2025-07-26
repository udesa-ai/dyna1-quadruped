import re
import numpy as np
import matplotlib.pyplot as plt

file_path = "data.txt"

# This regex matches everything between "tensor([[" and "]], device=...)" including newlines and spaces
pattern = re.compile(r'tensor\(\[\[(.*?)\]\], device=[^\)]*\)', re.DOTALL)

data_fields = {
    'base_lin_vel': [],
    'base_ang_vel': [],
    'projected_gravity': [],
    'velocity_commands': [],
    'joint_positions': [],
    'joint_velocities': [],
    'actions': []
}

with open(file_path, 'r') as f:
    content = f.read()

matches = pattern.findall(content)

for tensor_str in matches:
    # Clean up spaces, newlines and split by commas
    numbers_str = tensor_str.replace('\n', ' ').replace('  ', ' ')
    # Convert to list of floats
    values = np.fromstring(numbers_str, sep=',')

    if len(values) != 48:
        print(f"Warning: Skipping tensor with {len(values)} elements (expected 48).")
        continue

    data_fields['base_lin_vel'].append(values[0:3])
    data_fields['base_ang_vel'].append(values[3:6])
    data_fields['projected_gravity'].append(values[6:9])
    data_fields['velocity_commands'].append(values[9:12])
    data_fields['joint_positions'].append(values[12:24])
    data_fields['joint_velocities'].append(values[24:36])
    data_fields['actions'].append(values[36:48])

# Convert lists to numpy arrays
for key in data_fields:
    data_fields[key] = np.array(data_fields[key])

# Example output shapes
for key, arr in data_fields.items():
    print(f"{key}: {arr.shape}")


plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(data_fields['base_lin_vel'][:, 0], label='Base Linear Velocity X')
plt.plot(data_fields['base_lin_vel'][:, 1], label='Base Linear Velocity Y')
plt.plot(data_fields['base_lin_vel'][:, 2], label='Base Linear Velocity Z')
plt.title('Base Linear Velocity')
plt.xlabel('Time Step')
plt.ylabel('Velocity (m/s)')
plt.legend()   
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(data_fields['base_ang_vel'][:, 0], label='Base Angular Velocity X')
plt.plot(data_fields['base_ang_vel'][:, 1], label='Base Angular Velocity Y')
plt.plot(data_fields['base_ang_vel'][:, 2], label='Base Angular Velocity Z')
plt.title('Base Angular Velocity')
plt.xlabel('Time Step')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend()   
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(data_fields['projected_gravity'][:, 0], label='Projected Gravity X')
plt.plot(data_fields['projected_gravity'][:, 1], label='Projected Gravity Y')
plt.plot(data_fields['projected_gravity'][:, 2], label='Projected Gravity Z')
plt.title('Projected Gravity')
plt.xlabel('Time Step')
plt.ylabel('Gravity (m/s²)')
plt.legend()   
plt.grid()

plt.tight_layout()


# plot velocity commands
plt.figure(figsize=(12, 4))
plt.plot(data_fields['velocity_commands'][:, 0], label='Velocity Command X')
plt.plot(data_fields['velocity_commands'][:, 1], label='Velocity Command Y')
plt.plot(data_fields['velocity_commands'][:, 2], label='Velocity Command W')
plt.title('Velocity Commands')
plt.xlabel('Time Step')
plt.ylabel('Velocity Command (m/s)')
plt.legend()
plt.grid()
plt.tight_layout()

# plot joint positions
plt.figure(figsize=(12, 8))
for i in range(data_fields['joint_positions'].shape[1]):
    plt.plot(data_fields['joint_positions'][:, i], label=f'Joint Position {i+1}')
plt.title('Joint Positions')
plt.xlabel('Time Step')
plt.ylabel('Position (rad)')
plt.legend()
plt.grid()
plt.tight_layout()

# plot joint velocities
plt.figure(figsize=(12, 8))
for i in range(data_fields['joint_velocities'].shape[1]):
    plt.plot(data_fields['joint_velocities'][:, i], label=f'Joint Velocity {i+1}')
plt.title('Joint Velocities')
plt.xlabel('Time Step')
plt.ylabel('Velocity (rad/s)')
plt.legend()
plt.grid()
plt.tight_layout()


# plot actions
plt.figure(figsize=(12, 8))
for i in range(data_fields['actions'].shape[1]):
    plt.plot(data_fields['actions'][:, i], label=f'Action {i+1}')
plt.title('Actions')
plt.xlabel('Time Step')
plt.ylabel('Action Value')
plt.legend()
plt.grid()
plt.tight_layout()

plt.show()

print(data_fields['actions'][1, :])