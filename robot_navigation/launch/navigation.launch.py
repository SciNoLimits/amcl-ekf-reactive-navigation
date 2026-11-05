import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    robot_gazebo_dir = get_package_share_directory('robot_gazebo')
    robot_navigation_dir = get_package_share_directory('robot_navigation')

    map_file = os.path.join(
        robot_gazebo_dir,
        'maps',
        'room_map.yaml'
    )

    params_file = os.path.join(
        robot_navigation_dir,
        'config',
        'nav2_params.yaml'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'True',
            'autostart': 'True',
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])