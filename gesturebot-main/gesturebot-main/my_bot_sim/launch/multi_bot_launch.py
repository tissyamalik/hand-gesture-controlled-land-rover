from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_name = 'my_bot_sim'
    pkg_share = FindPackageShare(pkg_name).find(pkg_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'bot.urdf.xacro')

    def spawn_bot(robot_name, x, y):
        
        robot_description = ParameterValue(
            Command([
                'xacro ', xacro_file,
                ' robot_name:=', robot_name
            ]),
            value_type=str #explicit string type to fix error
        )

        return GroupAction([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=robot_name,
                name=f'{robot_name}_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description  
                }]
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', robot_name,
                    '-topic', f'/{robot_name}/robot_description',
                    '-x', str(x),
                    '-y', str(y),
                    '-z', '0.1',
                    '-robot_namespace', robot_name,
                ],
                output='screen'
            )
        ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
            ])
        ),

        spawn_bot('bot1', x=0.0, y=0.0),
        spawn_bot('bot2', x=2.0, y=0.0),
    ])