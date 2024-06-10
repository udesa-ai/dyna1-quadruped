from launch import LaunchDescription, actions
from launch_ros.actions import Node
from launch.actions import (IncludeLaunchDescription, RegisterEventHandler, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnShutdown
from launch.substitutions import LocalSubstitution
from pathlib import Path


def generate_launch_description():

    sharePath = get_package_share_directory('motor_can')
    dataLoc = os.path.join(Path(sharePath).parents[3], 'data')

    state_machine = Node(
            package='dyna_sm',
            namespace='',
            executable='state_machine',
            output="screen",
            name='state_machine',
            parameters=[
                {"frequency": 160.0}
            ])

    teleoperation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('teleoperation'),'launch'),'/teleop_launch.py']
        )
    )

    bag = Node(
            package='joint_listener',
            namespace='',
            executable='listener',
            name='listener',
            output="screen",
            parameters=[
                {"dataLocation": dataLoc}])

    joycon = Node(package='joy',
            namespace='',
            executable='joy_node',
            name='joy_node',
            output="screen",
            )
    
    control = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('controler'), 'launch'),
             			  '/real_interface_launch.py'
            ]
        )
    )

    
    return LaunchDescription([
        bag,
        joycon,
        # control,
        # state_machine,
        # teleoperation,
        ])
