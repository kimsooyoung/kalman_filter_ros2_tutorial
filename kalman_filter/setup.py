from glob import glob

import os

from setuptools import setup

package_name = 'kalman_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # TODO : models will imported through another methods => GAZEBO_ENV variable 
        # (os.path.join('share', package_name, 'models'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimsooyoung',
    maintainer_email='tge1375@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleport_service = kalman_filter.ros2_teleport_service:main',
            'odom_utility_tools = kalman_filter.ros2_odom_utility_tools:main',
            'laser_ray_localization = kalman_filter.ros2_laser_ray_localization:main',
            'kalman_1d = kalman_filter.ros2_kalman_filter:main',
        ],
    },
)
