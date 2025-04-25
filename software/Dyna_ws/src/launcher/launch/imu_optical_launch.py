from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    imu = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('bno055'), 'launch'),
             			  '/bno055.launch.py'
            ]
        )
    )

    optical = Node(package='serial_interface_py',
            namespace='',
            executable='serial_interface_py',
            name='serial_interface_py',
            output="screen",
            )
    
    return LaunchDescription([
        imu,
        optical
        ])
