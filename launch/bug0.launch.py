from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # turtlebot3_gazebo 패키지의 경로 가져오기
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # 실행할 기본 launch 파일 지정
    stage4_launch = os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_dqn_stage4.launch.py')

    return LaunchDescription([
        # Robot 모델 선택 인자 (burger, waffle, waffle_pi)
        DeclareLaunchArgument(
            'model',
            default_value='burger',
            description='Turtlebot3 model type'
        ),

        # Stage4 시뮬레이션 포함
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stage4_launch),
            launch_arguments={'model': LaunchConfiguration('model')}.items()
        ),

        Node(
            package ='bug0',
            executable = 'bug_move',
            name = 'bug_move',
            output = 'screen',
        )
    ])