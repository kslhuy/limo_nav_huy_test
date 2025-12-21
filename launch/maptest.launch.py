import os
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    # package paths
    pkg_share = get_package_share_path('limo_nav_huy_test')
    pkg_description = get_package_share_path('limo_description')

    amcl_params = str(pkg_share / 'param' / 'amcl_params.yaml')
    rviz_config = str(pkg_share / 'rviz' / 'nav2_copy.rviz')
    model_path = str(pkg_description / 'urdf' / 'limo_ackerman.xacro')

    # robot_description from xacro
    robot_description = ParameterValue(Command(['xacro ', model_path]), value_type=str)

    # IMPORTANT: Run bringup first!
    #   ros2 launch limo_bringup limo_start.launch.py
    # Then run this file to start navigation and RViz.

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': '/home/agilex/agilex_ws/bib_cran_map.yaml'}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[amcl_params],
            output='screen'
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]
        ),

        Node(
            package='limo_nav_huy_test',
            executable='waypoints',
            name='waypoints',
            parameters=[{
                'nodeSequence': [10, 2, 4, 6, 8, 10],
                'dx': 0.7,
                'dy': 2.0,
                'theta_deg': 80.0,
                'scale': 1.0,
                'mirror_x': False,
                'mirror_y': False,
                'swap_xy': False
            }],
            output='screen'
        ),

        # Robot model publisher - NEEDED because limo_start doesn't provide it!
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
            output='screen'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])