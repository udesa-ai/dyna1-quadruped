path = '/home/udesa/Downloads/tracing/my-tracing-session/ust'

import json
import sys
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from os.path import exists

from tracetools_analysis.loading import load_file


if not exists(path+"/raw.json"):
    print('Does not exist')
    events = load_file(path)
    with open(path+"/raw.json", 'w') as final:
        json.dump(events, final)
    
    df = pd.DataFrame(events)
else:
    print('Already exists')
    df = pd.read_json(path+"/raw.json")


print(df['_name'].unique())
print(df['procname'].unique())

# rinterface = df[df['procname'] == 'rinterface']

# can_interface = df[df['procname'] == 'ros2can_bridge']

# callback = df[df['_name'] == 'ros2:callback_start']

# print(callback['procname'].unique())
# print()
# print(rinterface['_name'].unique())