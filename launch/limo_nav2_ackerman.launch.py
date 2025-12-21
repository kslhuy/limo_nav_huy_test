import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch.logging


def generate_launch_description():
    logger = launch.logging.get_logger("log")

    limo_bringup_dir = get_package_share_directory('limo_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # map_yaml_path = LaunchConfiguration('map',default=os.path.join(limo_bringup_dir,'maps','map02.yaml'))
    # map_yaml_path = LaunchConfiguration('map',default=os.path.join(limo_bringup_dir,'maps','mapGEII.yaml'))
    map_yaml_path = LaunchConfiguration('map',default=('/home/agilex/agilex_ws/bib_cran_map.yaml'))
    # map_yaml_path = LaunchConfiguration('map',default=('/home/maphuy.yaml'))
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(limo_bringup_dir,'param','nav2.yaml'))
    # nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(limo_bringup_dir,'param','navigation2.yaml'))

    rviz_config_dir = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value=use_sim_time,description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map',default_value=map_yaml_path,description='Full path to map file to load'),
        DeclareLaunchArgument('params_file',default_value=nav2_param_path,description='Full path to param file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([limo_bringup_dir,'/launch/humble','/bringup.launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
