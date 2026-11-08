import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    robot_gazebo_dir = get_package_share_directory('robot_gazebo')
    robot_navigation_dir = get_package_share_directory('robot_navigation')

    map_file = os.path.join(
        robot_gazebo_dir,
        'maps',
        'room_map.yaml'
    )

    # [P2] Nav2 params file is now a launch argument. Default: the tuned AMCL file.
    # Baseline (stock AMCL):  params_file:=<share>/robot_navigation/config/nav2_params.yaml
    default_params_file = os.path.join(
        robot_navigation_dir,
        'config',
        'nav2_params_amcl_tuned.yaml'
    )
    params_file = LaunchConfiguration('params_file')

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
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the Nav2 parameters file'),  # [P2]
        nav2_launch
    ])
