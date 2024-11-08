"""Starts all the nodes to visualize a robot in rviz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import \
    Command, EqualsSubstitution, PathJoinSubstitution, LaunchConfiguration, NotEqualsSubstitution
from launch_ros.actions import Node

from launch.conditions import IfCondition

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the robot state publisher, joint state publisher, and rviz."""
    return LaunchDescription([
        DeclareLaunchArgument(name='view_only', default_value='false',
                              description='Only launch rviz'),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d',
                       PathJoinSubstitution(
                           [FindPackageShare('diff_drive'), 'view.rviz'])],
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    'xacro ',
                    FindPackageShare('diff_drive'),
                    '/ddrive.urdf.xacro'
                ]),
            }]
        ),

        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui',
             condition=IfCondition(EqualsSubstitution(
                 LaunchConfiguration('view_only'), 'true'))
             ),

        Node(package='joint_state_publisher',
             executable='joint_state_publisher',
             name='joint_state_publisher',
             condition=IfCondition(NotEqualsSubstitution(
                    LaunchConfiguration('view_only'), 'true'))
             ),
    ])
