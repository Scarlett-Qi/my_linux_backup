import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    package_path = get_package_share_directory('gr_planning')
    config_path = os.path.join(package_path, 'config', 'ForkliftPath.yaml')

    info = LogInfo(msg="Launch Forklift Planning.")

    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=config_path,
        description='Yaml config file path'
    )

    detect_server = Node(package='gr_planning',
                           executable='gr_planning_forklift_correct_server',
                           parameters=[config_path],
                           output='screen')

    ld = LaunchDescription()

    ld.add_action(detect_server)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(info)

    return ld