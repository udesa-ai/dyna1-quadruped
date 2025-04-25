import launch
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('motor_can'),
        'config',
        'motor_params.yaml'
        )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    motores = Node(
            package='motor_can',
            namespace='',
            executable='motores',
            name='motores',
            output="screen",
            parameters=[
                config,
                {'use_sim_time': use_sim_time}
            ])
    
    return launch.LaunchDescription([motores])