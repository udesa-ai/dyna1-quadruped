from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import launch.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.events.process import ProcessStarted

def generate_launch_description():
    MAX_CURRENT = LaunchConfiguration('MAX_CURRENT')
    MAX_CURRENT_launch_arg = launch.actions.DeclareLaunchArgument(
        'MAX_CURRENT',
        default_value='30'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    inter = Node(
            package='can_interface',
            namespace='',
            executable='interface',
            name='CAN_Interfacec',
            parameters=[{'use_sim_time': use_sim_time}],
            output="screen")

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
            parameters=[{'use_sim_time': use_sim_time}],
            output="screen")
 

    control = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('controler_cpp'), 'launch'),
             			  '/real_interface_launch.py'
            ]
        ),
        launch_arguments={'MAX_CURRENT':MAX_CURRENT}.items()
    )

    imu = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('bno055'), 'launch'),
             			  '/bno055.launch.py'
            ]
        )
    )

    safety = Node(
            package='safety',
            namespace='',
            executable='precautions',
            name='Precautions',
            parameters=[{'use_sim_time': use_sim_time}],
            output="screen")
    
    optical = Node(package='serial_interface_py',
            namespace='',
            executable='serial_interface_py',
            name='serial_interface_py',
            output="screen",
            )

    net = Node(package='neural_net',
            namespace='',
            executable='neural_thinking',
            name='Neural_Net',
            output="screen",
            )
    
    return LaunchDescription([
        MAX_CURRENT_launch_arg,
        inter,
        imu,
        motores,
        can_bridge,
        control,
        safety,
        optical,
        net])
