"""Launch description for Gazebo simulation, Nav2 stack, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Gazebo, Nav2, and RViz."""
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Use the package share directory (portable: works after 'colcon build' on any machine)
    # Maps and config are installed via setup.py data_files into the package share.
    pkg_share = get_package_share_directory('nav2_goal_navigator')
    default_map_path = os.path.join(pkg_share, 'maps', 'map.yaml')
    default_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default=default_map_path)
    params_file = LaunchConfiguration('params_file', default=default_params_path)

    # 1. Gazebo simulation launch
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 2. Nav2 stack launch
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items()
    )

    # 3. RViz2 launch with Nav2 configuration
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    ))
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=default_map_path, description='Full path to map file'
    ))
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=default_params_path, description='Full path to Nav2 params'
    ))
    ld.add_action(gazebo_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(rviz_cmd)

    return ld
