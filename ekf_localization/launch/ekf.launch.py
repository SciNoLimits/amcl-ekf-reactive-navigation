import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ekf_localization')
    config = os.path.join(share, 'config', 'ekf.yaml')

    imu_adapter = Node(
        package='ekf_localization',
        executable='imu_covariance_adapter',
        name='imu_covariance_adapter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'imu_frame': 'imu_link',
            'angular_velocity_variance': 1.0e-4,
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([imu_adapter, ekf])
