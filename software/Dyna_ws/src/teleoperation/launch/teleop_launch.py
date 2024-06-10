import launch
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # joystick = Node(
    #              package="joy",
    #              namespace='',
    #              executable='joy_node',
    #              name='joycon'
    #            )
    
    teleop = Node(
            package='teleoperation',
            namespace='',
            executable='teleoperator',
            name='teleoperator',
            parameters=[
                {"frequency": 100.0,
                 "axis_linear_x":4,
                 "axis_linear_y":3,
                 "axis_linear_z":1,
                 "axis_angular":0,
                 "scale_linear":1.0,
                 "scale_angular":1.0,
                 "button_switch":0,
                 "button_estop":1,
                 "start_button":7}
            ])
  
    
    return launch.LaunchDescription([teleop]) #, joystick])