from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    robot_gazebo_launch = PathJoinSubstitution([FindPackageShare("robot_gazebo"), "launch", "gazebo.launch.py"])
    robot_navigation_launch = PathJoinSubstitution([FindPackageShare("robot_gazebo"), "launch", "gazebo.launch.py"])
    
    
    return LaunchDescription([
        
    ])