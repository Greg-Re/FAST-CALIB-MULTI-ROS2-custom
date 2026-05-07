from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('fast_calib')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'qr_params.yaml'])

    multi_calib_node = Node(
        package='fast_calib',
        executable='multi_fast_calib',
        name='multi_fast_calib',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        multi_calib_node
    ])
