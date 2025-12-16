from setuptools import setup

package_name = 'limo_nav_huy_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'vehicle_main_ros = limo_nav_huy_test.vehicle_main_ros:main'
        ],
    },
)
