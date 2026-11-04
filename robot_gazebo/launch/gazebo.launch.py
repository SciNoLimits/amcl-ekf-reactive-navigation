from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    description = PathJoinSubstitution([FindPackageShare("robot_gazebo"), "urdf", "robot_gazebo.urdf.xacro"])
    default_world = PathJoinSubstitution([FindPackageShare("robot_gazebo"), "worlds", "empty.sdf"])
    controllers = PathJoinSubstitution([FindPackageShare("robot_gazebo"), "config", "controllers.yaml"])
    bridge_config = PathJoinSubstitution([FindPackageShare("robot_gazebo"), "config", "bridge.yaml"])
    gazebo_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])

    description_resource_root = PathJoinSubstitution([
        FindPackageShare("robot_description"),
        ".."
    ])

    set_gazebo_resources = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            description_resource_root,
            ":",
            EnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                default_value=""
            )
        ],
    )
    
    robot_description = ParameterValue(
        Command(["xacro ", description]),
        value_type=str,
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={"gz_args": ["-r -v 3 ", world]}.items(),
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "robot",
            "-topic",
            "robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.15",
        ],
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )
    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    diff = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription([
        set_gazebo_resources,
        DeclareLaunchArgument("world", default_value=default_world, description="Gazebo SDF world"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        gazebo, rsp, bridge,
        TimerAction(period=3.0, actions=[spawn]),
        TimerAction(period=6.0, actions=[jsb, diff]),
    ])
