path = '/home/udesa/Downloads/my-tracing-session/ust'


import sys

sys.path.insert(0, '../')
sys.path.insert(0, '../../../ros-tracing/ros2_tracing/tracetools_read/')

import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.express as px

import datetime as dt

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from tracetools_analysis.loading import load_file
from tracetools_analysis.processor.ros2 import Ros2Handler
from tracetools_analysis.utils.ros2 import Ros2DataModelUtil

# Process
events = load_file(path)

handler = Ros2Handler.process(events)

data_util = Ros2DataModelUtil(handler.data)

callback_symbols = data_util.get_callback_symbols()


# Initialize figure
fig = go.Figure()
fig = make_subplots(rows=1, cols=2)

names = []
for obj, symbol in callback_symbols.items():
    owner_info = data_util.get_callback_owner_info(obj)
    names.append(owner_info)
    if owner_info is None:
        owner_info = '[unknown]'

    # Filter out internal subscriptions
    if '/parameter_events' in owner_info:
        continue

    # Duration
    duration_df = data_util.get_callback_durations(obj)
    starttime = duration_df.loc[:, 'timestamp'].iloc[0].strftime('%Y-%m-%d %H:%M')

    duration = np.array(duration_df['duration']).astype(int)/1000000

    fig.add_trace(
        go.Scatter(
            x = duration_df['timestamp'],
            y = duration,
            name = owner_info,
            visible = False
        ), row = 1, col = 1)
    
    fig.add_trace(
        go.Histogram(
            x = duration,
            name = owner_info,
            visible = False
        ), row = 1, col = 2)


fig.update_xaxes(title_text="time", row=1, col=1)
fig.update_xaxes(title_text="duration [ms]", row=1, col=2)

fig.update_yaxes(title_text="duration [ms]", row=1, col=1)
fig.update_yaxes(title_text="frequency", type="log", row=1, col=2)

button = []
base = [False, False] * len(names)

names.append('All')

for index, name in enumerate(names):
    temp = base.copy()
    temp[index*2:index*2+2] = [True, True]

    if name == 'All':
        temp = [True]*len(temp)

    button.append(
        dict(label = name,
             method = 'update',
             args = [{'visible': temp},
                     {'title': name}]
            ))


fig.update_layout(
    showlegend=False,
    updatemenus=[
        dict(
            active=0,
            buttons = button,
            x = 0.3,
            xanchor = 'left',
            y = 1.2,
            yanchor = 'top',
        )
    ])

# Set title
fig.update_layout(title_text="ROS2 Trace")

fig.show()