import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    limo_bringup_dir = get_package_share_directory('limo_bringup')
    limo_nav_huy_test_dir = get_package_share_directory('limo_nav_huy_test')
    
    # Same configs as limo_nav2.launch.py
    use_sim_time = 'false'
    map_yaml_path = '/home/agilex/agilex_ws/bib_cran_map.yaml'
    nav2_param_path = os.path.join(limo_nav_huy_test_dir, 'param', 'nav2.yaml')
    rviz_config_dir = os.path.join(limo_nav_huy_test_dir, 'rviz', 'nav2.rviz')
    
    return LaunchDescription([
        # Include bringup (robot + nav2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([limo_bringup_dir, '/launch/humble', '/bringup.launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path
            }.items(),
        ),
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        # Your nodes
        Node(package='limo_nav_huy_test', executable='waypoints', name='waypoints'),
        Node(package='limo_nav_huy_test', executable='vehicle_main_ros', name='vehicle_main_ros')
    ])