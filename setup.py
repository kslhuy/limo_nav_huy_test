from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'limo_nav_huy_test'

# Function to recursively get all files in a directory
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', path, filename))
    return paths

# Get all files from multi_vehicle_RealCar directory
extra_files = package_files('limo_nav_huy_test/multi_vehicle_RealCar')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_data={
        package_name: ['multi_vehicle_RealCar/**/*', 'multi_vehicle_RealCar/**/**/*'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/param', glob('param/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wandering = limo_nav_huy_test.wandering:main',
            'vehicle_main_ros = limo_nav_huy_test.vehicle_main_ros:main',
            'waypoints = limo_nav_huy_test.waypoints:main',
            # 'smart_goal_sender = limo_nav_huy_test.SmartGoalSender:main',
            # QCar coordinate system versions
            'vehicle_main_ros_qcar = limo_nav_huy_test.vehicle_main_ros_qcar:main',
            'waypoints_qcar = limo_nav_huy_test.waypoints_qcar:main',
            'waypoint_alignment_helper = limo_nav_huy_test.waypoint_alignment_helper:main',
        ],
    },
)
