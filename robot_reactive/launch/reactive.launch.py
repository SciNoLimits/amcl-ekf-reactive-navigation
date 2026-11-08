"""Start the Person 4 reactive layer (obstacle detector + reactive controller)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('robot_reactive'), 'config',
                                   'reactive_params.yaml'])
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    enabled = ParameterValue(LaunchConfiguration('enable_reactive'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'enable_reactive', default_value='true',
            description='false = forward Nav2 commands unchanged (for comparison runs)'),
        Node(package='robot_reactive', executable='obstacle_detector', output='screen',
             parameters=[params, {'use_sim_time': use_sim_time}]),
        Node(package='robot_reactive', executable='reactive_controller', output='screen',
             parameters=[params, {'use_sim_time': use_sim_time, 'enabled': enabled}]),
    ])
