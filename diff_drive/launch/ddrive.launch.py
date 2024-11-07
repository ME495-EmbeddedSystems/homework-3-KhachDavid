from launch import LaunchDescription

from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch Gazebo server and client.

    Returns:
        LaunchDescription: The launch description object

    """
    return LaunchDescription([
        Node(
            package='ros_gz_sim',
            executable='gzserver',
            output='screen',
            parameters=[{
                'world_sdf_file': PathJoinSubstitution([
                    FindPackageShare('diff_drive'), 'worlds', 'ddrive.world.sdf'
                ]),
                'use_sim_time': True
            }]
        ),
    ])
