import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

import os


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur", package_name="ur5_moveit_config").to_moveit_configs()
    declared_arguments = [] 
    # cycle_move = LaunchConfiguration('cycle_move')
    cycle_move_launch_arg = DeclareLaunchArgument(
        'cycle_move',
        default_value='false'
    )
    declared_arguments.append(cycle_move_launch_arg)
    launch_autograde = LaunchConfiguration('autograde')
    launch_autograde_arg = DeclareLaunchArgument(
        'autograde',
        default_value='false'
    )
    declared_arguments.append(launch_autograde_arg)
    # state_publisher = LaunchConfiguration('state_publisher')
    state_publisher_launch_arg = DeclareLaunchArgument(
        'state_publisher',
        default_value='true'
    )
    declared_arguments.append(state_publisher_launch_arg)
    # rviz = LaunchConfiguration('rviz')
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true'
    )
    declared_arguments.append(rviz_arg)

    rd_file = DeclareLaunchArgument(
        'rd_file',
        default_value=os.path.join(get_package_share_directory('assignment3'), "urdf", "ur5_urdf.xml")
    )
    declared_arguments.append(rd_file)

    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
            default_value="ur5",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    launch_description = generate_move_group_launch(moveit_config)
    launch_description.add_entity(launch.LaunchDescription(declared_arguments + 
        [
        launch_ros.actions.Node(
            package='robot_sim',
            executable='robot_sim_bringup',
            name='cycle_move',
            parameters=[
                {'cycle_move': LaunchConfiguration('cycle_move')},
                {"rd_file": LaunchConfiguration('rd_file')}
            ],
            output='screen'),    
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
            condition=IfCondition(LaunchConfiguration('state_publisher'))),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=["-d", rviz_config_file],),
        launch_ros.actions.Node(
            package='assignment3',
            executable='marker_control',
            name='marker_control',
            parameters=[{"rd_file": LaunchConfiguration('rd_file')}],
            output='screen',),
        launch_ros.actions.Node(
            package='assignment3',
            executable='move_arm',
            name='move_arm',
            parameters=[{"rd_file": LaunchConfiguration('rd_file')}],
            output='screen',),
        launch_ros.actions.Node(
            package='obstacle_controller',
            executable='obstacle_controller',
            name='obstacle_controller',
            output='screen',),
        launch_ros.actions.Node(
            package='assignment3',
            executable='autograde',
            name='autograde',
            parameters=[{"rd_file": LaunchConfiguration('rd_file')}],
            condition=IfCondition(LaunchConfiguration('autograde')),
            output='screen'),]))

    return launch_description
