import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo_launch = PathJoinSubstitution([FindPackageShare("robot_gazebo"), "launch", "gazebo.launch.py"])
    navigation_launch = PathJoinSubstitution([FindPackageShare("robot_navigation"), "launch", "navigation.launch.py"])
    nav2_rviz_config = os.path.join(
        get_package_share_directory("robot_navigation"),
        "rviz",
        "nav2_rviz_config.rviz"
    )
    
    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch)
    )
    
    robot_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch)
    )
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output='screen',
        arguments=['-d', nav2_rviz_config],
        parameters=[{'use_sim_time': True}]
        
    )
    
    return LaunchDescription([
        robot_gazebo_launch,
        TimerAction(period=3.0, actions=[robot_navigation_launch]),
        TimerAction(period=6.0, actions=[rviz2])
        
    ])