from launch import LaunchDescription, actions
from launch_ros.actions import Node
from launch.actions import (IncludeLaunchDescription, RegisterEventHandler, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnShutdown
from launch.substitutions import LocalSubstitution


def generate_launch_description():
    motores = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('motor_can'),'launch'),'/motor_launch.py']
        )
    )

    can_bridge = Node(
            package='ros2socketcan_bridge',
            namespace='',
            executable='ros2can_bridge',
            name='CANbridge',
            output="screen")
    
    # can_bridge = actions.ExecuteProcess(
    #     cmd=['ros2can_bridge']
    # )
    

    control = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('controler'), 'launch'),
             			  '/control_launch.py'
            ]
        )
    )

    keyboard = Node(
            package='keyboard',
            namespace='',
            executable='keyboard',
            name='keyboard')
    
    return LaunchDescription([
        can_bridge,
        motores,
        control,
        keyboard
        ])