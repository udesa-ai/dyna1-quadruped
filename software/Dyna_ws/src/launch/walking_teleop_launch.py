from launch import LaunchDescription, actions
from launch_ros.actions import Node
from launch.actions import (IncludeLaunchDescription, RegisterEventHandler, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnShutdown
from launch.substitutions import LocalSubstitution
from pathlib import Path
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    motores = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('motor_can'),'launch'),'/motor_launch.py']
        )
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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
                os.path.join(get_package_share_directory('controler'), 'launch'),
             			  '/real_interface_launch.py'
            ]
        )
    )

    imu = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('mpu6050driver'), 'launch'),
             			  '/mpu6050driver_launch.py'
            ]
        )
    )

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

    sharePath = get_package_share_directory('motor_can')
    dataLoc = os.path.join(Path(sharePath).parents[3], 'data')

    bag = Node(
            package='joint_listener',
            namespace='',
            executable='listener',
            name='listener',
            parameters=[
                {"dataLocation": dataLoc}])
    
    imu_translator = Node(
            package='imu_translator',
            namespace='',
            executable='translator',
            name='translator')

    
    return LaunchDescription([
        # imu,
        motores,
        can_bridge,
        # control,
        # teleoperation,
        # state_machine,
        # bag,
        # imu_translator
        ])
