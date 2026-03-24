import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_path = get_package_share_directory('mfja_3rd_floor_gz')
    base_launch = os.path.join(pkg_path, 'launch', 'mfja_3rdf_kuka.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robots',
            default_value='',
            description=(
                'Comma-separated robot selection list. Supports full names, '
                'short aliases, numeric indices, or "all".'
            ),
        ),
        DeclareLaunchArgument(
            'robot_config',
            default_value='config/robots_room_315_only.yaml',
            description='Robot spawn YAML for the lightweight room 315 mode.',
        ),
        DeclareLaunchArgument(
            'gz_partition',
            default_value=f'room_315_only_{os.getpid()}',
            description='Gazebo transport partition used to isolate this launch instance.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            choices=['true', 'false'],
            description='Start Gazebo GUI client.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'world_name': 'room_315_only',
                'robot_config': LaunchConfiguration('robot_config'),
                'robots': LaunchConfiguration('robots'),
                'gz_partition': LaunchConfiguration('gz_partition'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'gui': LaunchConfiguration('gui'),
            }.items(),
        ),
    ])
