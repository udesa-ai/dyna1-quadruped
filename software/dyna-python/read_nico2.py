import re
import numpy as np
import torch
import torch.nn as nn

# Define the model architecture
class ActorMLP(nn.Module):
    def __init__(self):
        super(ActorMLP, self).__init__()
        self.model = nn.Sequential(
            nn.Linear(48, 128),
            nn.ELU(alpha=1.0),
            nn.Linear(128, 128),
            nn.ELU(alpha=1.0),
            nn.Linear(128, 128),
            nn.ELU(alpha=1.0),
            nn.Linear(128, 12)
        )

    def forward(self, x):
        return self.model(x)


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

checkpoint = torch.load("/home/dynabot/model_4350.pt") #, map_location=torch.device('cpu'))
model_state_dict = checkpoint['model_state_dict']
actor_state_dict = {k.replace('actor.', 'model.'): v for k, v in model_state_dict.items() if k.startswith('actor.')} 

model = ActorMLP()
model.load_state_dict(actor_state_dict)
model.eval()

for i in range(len(data_fields['base_lin_vel'])):
    input_data = np.zeros(48)
    input_data[0:3] = data_fields['base_lin_vel'][i]
    input_data[3:6] = data_fields['base_ang_vel'][i]
    input_data[6:9] = data_fields['projected_gravity'][i]
    input_data[9:12] = data_fields['velocity_commands'][i]
    input_data[12:24] = data_fields['joint_positions'][i]
    input_data[24:36] = data_fields['joint_velocities'][i]
    input_data[36:48] = data_fields['actions'][i]

    input_data = [float(x) for x in input_data]
    output = model(torch.tensor([input_data])).squeeze(0).tolist()
    actions = output

    print(actions)