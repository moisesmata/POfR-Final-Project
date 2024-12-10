import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('cycle_move', default_value='false'),
        launch.actions.DeclareLaunchArgument('state_publisher', default_value='false'),
        launch.actions.DeclareLaunchArgument('rviz', default_value='false'),
        
        launch.actions.DeclareLaunchArgument('robot_description',
            default_value="$(find xacro)/xacro '$(find lwr_defs)/robots/kuka_lwr_arm.urdf.xml'"),
        
        launch_ros.actions.Node(
            package='robot_sim',
            executable='robot_sim_bringup',
            name='robot_sim',
            output='screen',
            parameters=[{'cycle_move': launch.substitutions.LaunchConfiguration('cycle_move')}]
        ),
        
        launch.conditionals.IfCondition(
            launch.substitutions.LaunchConfiguration('state_publisher')
        ).then(
            launch_ros.actions.Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen'
            )
        ),
        
        launch.conditionals.IfCondition(
            launch.substitutions.LaunchConfiguration('rviz')
        ).then(
            launch_ros.actions.Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                output='screen',
                arguments=['-d', os.path.join(
                    os.path.dirname(os.path.abspath(__file__)),
                    'robot_sim', 'config', 'lwr.rviz')]
            )
        ),
    ])
