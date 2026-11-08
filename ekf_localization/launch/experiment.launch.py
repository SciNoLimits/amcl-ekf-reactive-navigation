from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    output_dir = LaunchConfiguration('output_dir')

    return LaunchDescription([
        DeclareLaunchArgument('goal_x', description='Reachable goal x in map frame'),
        DeclareLaunchArgument('goal_y', description='Reachable goal y in map frame'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('output_dir', default_value='ekf_experiment'),

        Node(
            package='ekf_ground_truth',
            executable='gazebo_ground_truth_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'gazebo_topic': '/world/default/dynamic_pose/info',
                'entity_name': 'robot',
                'frame_id': 'world',
            }],
        ),

        Node(
            package='ekf_localization',
            executable='run_ekf_experiment',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '--goal-x', goal_x,
                '--goal-y', goal_y,
                '--goal-yaw', goal_yaw,
                '--goal-frame', 'map',
                '--output-dir', output_dir,
            ],
        ),
    ])
