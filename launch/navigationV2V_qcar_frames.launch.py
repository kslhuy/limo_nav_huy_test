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
    rviz_config_dir = os.path.join(limo_nav_huy_test_dir, 'rviz', 'nav2_copy.rviz')
    
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
        
        # QCar-style coordinate frame: static transform map -> map_rotated
        # This creates the map_rotated frame that QCar uses
        # Identity transform (no rotation/translation) by default
        # Adjust translation/rotation via launch arguments if needed
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_map_rotated',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'map_rotated'],
            output='screen'
        ),
        
        # QCar-style nodes (using map_rotated frame)
        Node(package='limo_nav_huy_test', executable='waypoints_qcar', name='waypoints_qcar'),
        Node(package='limo_nav_huy_test', executable='vehicle_main_ros_qcar', name='vehicle_main_ros_qcar'),
        
        # RViz (run separately to avoid crashes blocking other nodes)
        # Run manually: ros2 run rviz2 rviz2 -d /home/agilex/agilex_ws/install/limo_nav_huy_test/share/limo_nav_huy_test/rviz/nav2_copy.rviz
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'
        # ),
    ])
