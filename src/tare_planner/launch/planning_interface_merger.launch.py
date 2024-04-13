from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_merger(context):
  config_file = get_package_share_directory('tare_planner') + '/planning_interface_merger.yaml'
  namespace = LaunchConfiguration('namespace')
  delay_time = int(LaunchConfiguration('delay_time').perform(context))
  num_robots = int(LaunchConfiguration('num_robots').perform(context))
  node = Node(
      package='tare_planner',
      executable='planning_interface_merger',
      name='planning_interface_merger',
      output='screen',
      parameters=[config_file, {
          'delay_in_seconds': delay_time,
          'num_robots': num_robots
      }],
      namespace=namespace,
  )
  return [node]


def generate_launch_description():
  declare_namespace = DeclareLaunchArgument(
      'namespace',
      default_value='default',
      description='Namespace for the node',
  )
  declare_delay_time = DeclareLaunchArgument(
      'delay_time',
      default_value='5',
      description='',
  )
  declare_num_robots = DeclareLaunchArgument(
      'num_robots',
      default_value='2',
      description='',
  )

  return LaunchDescription([
      declare_namespace,
      declare_delay_time,
      declare_num_robots,
      OpaqueFunction(function=launch_merger),
  ])
