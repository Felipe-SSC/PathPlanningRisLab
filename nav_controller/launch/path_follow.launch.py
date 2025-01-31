# Authors: Abdulkadir Ture
# Github : abdulkadrtr
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    rviz_file = os.path.join(get_package_share_directory('nav_controller'), 'config', 'rviz_config.rviz')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='async_slam_toolbox_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='nav_controller',
            executable='control',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])