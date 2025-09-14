from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the path of the turtlebot3_gazebo package
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

     # Path to the default stage4 launch file
    stage4_launch = os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_dqn_stage4.launch.py')

    return LaunchDescription([
        # Argument to select robot model (burger, waffle, waffle_pi)
        DeclareLaunchArgument(
            'model',
            default_value='burger',
            description='Turtlebot3 model type'
        ),

        # Include stage4 simulation launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stage4_launch),
            launch_arguments={'model': LaunchConfiguration('model')}.items()
        ),
        # Launch the Bug0 navigation node
        Node(
            package ='bug0',
            executable = 'bug_move',
            name = 'bug_move',
            output = 'screen',
        )
    ])