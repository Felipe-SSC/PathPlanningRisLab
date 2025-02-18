import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Lanzar Gazebo con el mundo específico
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'tree_test2.launch.py')
        ),
    )

    mapeo = Node(
        package='nav_controller',
        executable='mapeo2',
        name='mappingNode',
    )

    # Lanzar RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[],
        output='screen'
    )

    # Lanzar SLAM Toolbox en modo *online*
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',  # Usa 'sync_slam_toolbox_node' si prefieres el modo síncrono
        name='slam_toolbox',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        rviz2,
        slam_toolbox,
        mapeo,
    ])
