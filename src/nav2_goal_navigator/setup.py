"""Setup script for nav2_goal_navigator package."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'nav2_goal_navigator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required: register this package in ament's index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Required: install package.xml for rosdep and ros2 pkg tools
        ('share/' + package_name, ['package.xml']),
        # Install launch files so 'ros2 launch nav2_goal_navigator ...' works
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install maps into package share (map.yaml + map.pgm)
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        # Install Nav2 parameter file into package share
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sanket',
    maintainer_email='sanketgombi@gmail.com',
    description='ROS 2 Python node commanding autonomous navigation to a target pose via Nav2',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'navigate_to_goal = nav2_goal_navigator.navigate_to_goal:main',
        ],
    },
)
