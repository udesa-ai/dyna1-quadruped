from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    MAX_CURRENT = LaunchConfiguration('MAX_CURRENT')
    MAX_CURRENT_launch_arg = DeclareLaunchArgument(
        'MAX_CURRENT',
        default_value='30'
    )
    

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')


    uart_bridge = Node(
            package='uart_bridge',
            namespace='',
            executable='uart_bridge_node',
            name='UARTbridge',
            parameters=[{'use_sim_time': use_sim_time}],
            output="screen")

    motores = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('motor_can'),'launch'),'/motor_launch.py']
        )
    )

    control = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('controler_cpp'), 'launch'),
             			  '/real_interface_launch.py'
            ]
        ),
        launch_arguments={'MAX_CURRENT':MAX_CURRENT}.items()
    )

    safety = Node(
            package='safety',
            namespace='',
            executable='precautions',
            name='Precautions',
            parameters=[{'use_sim_time': use_sim_time}],
            output="screen")
    
    # Register event handler to launch these only after uart_bridge has started
    launch_rest = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=uart_bridge,
            on_start=[
                motores,
                control,
                safety
            ]
        )
    )

    return LaunchDescription([
        MAX_CURRENT_launch_arg,
        uart_bridge,
        launch_rest
    ])
