import launch
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    MAX_CURRENT = LaunchConfiguration('MAX_CURRENT')

    MAX_CURRENT_launch_arg = DeclareLaunchArgument(
        'MAX_CURRENT',
        default_value='10'
    )

    config_dyna = os.path.join(
        get_package_share_directory('controler_cpp'),
        'config',
        'dyna_params.yaml'
        )
    
    config_joycon = os.path.join(
        get_package_share_directory('controler_cpp'),
        'config',
        'joy_params.yaml'
        )
    
    control = Node(
            package='controler_cpp',
            namespace='',
            executable='interface',
            name='interface',
            output="screen",
            emulate_tty=True,
            parameters=[config_dyna, config_joycon, {"MAX_CURRENT" : MAX_CURRENT} ])
    
    return launch.LaunchDescription([
        MAX_CURRENT_launch_arg,
        control,
  ])