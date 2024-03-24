from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = get_package_share_directory('tare_planner') + '/planning_interface_merger.yaml'
    namespace = LaunchConfiguration('namespace')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='default',
        description='Namespace for the node',
    )

    return LaunchDescription([
        declare_namespace,
        Node(
            package='tare_planner',
            executable='planning_interface_merger',
            name='planning_interface_merger',
            output='screen',
            parameters=[config_file],
            namespace=namespace,
        ),
    ]) 
