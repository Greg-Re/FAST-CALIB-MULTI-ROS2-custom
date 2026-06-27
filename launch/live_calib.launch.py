from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    pkg_share   = FindPackageShare('fast_calib')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'qr_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz_cfg', 'fast_livo2.rviz'])

    # Node name must match the 'fast_calib:' section in qr_params.yaml so that
    # all shared camera / target / topic parameters are loaded automatically.
    live_calib_node = Node(
        package='fast_calib',
        executable='live_fast_calib',
        name='fast_calib',
        output='screen',
        parameters=[params_file]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        live_calib_node,
        rviz_node,
    ])
