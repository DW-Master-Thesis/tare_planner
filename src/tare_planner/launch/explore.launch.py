from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_tare_node(context):
  config_file = str(LaunchConfiguration('configFile').perform(context))
  robot_name = str(LaunchConfiguration('robotName').perform(context))
  robot_id = int(robot_name.split('_')[-1])
  namespace = str(LaunchConfiguration('namespace').perform(context))
  rviz = LaunchConfiguration('rviz')
  tare_planner_node = Node(
      package='tare_planner',
      executable='tare_planner_node',
      name='tare_planner_node',
      output='screen',
      parameters=[
          get_package_share_directory('tare_planner') + "/" + config_file + '.yaml',
          {
              "kRobotId": int(robot_id),
              "kNumRobots": int(LaunchConfiguration('numRobots').perform(context)),
              "global_namespace": '/' + namespace + '/'
          },
      ],
      namespace=namespace + '/' + robot_name,
  )
  rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      name='tare_planner_ground_rviz',
      arguments=['-d', get_package_share_directory('tare_planner') + '/tare_planner_ground.rviz'],
      namespace=namespace + '/' + robot_name,
      condition=IfCondition(rviz),
  )
  return [rviz_node, tare_planner_node]


def generate_launch_description():

  declare_configFile = DeclareLaunchArgument(
      'configFile',
      default_value='default',
      description='',
  )
  declare_namespace = DeclareLaunchArgument(
      'namespace',
      default_value='default',
      description='Namespace for the node',
  )
  declare_robotName = DeclareLaunchArgument(
      'robotName',
      default_value='robot_0',
      description='',
  )
  declare_rviz = DeclareLaunchArgument(
      'rviz',
      default_value='false',
      description='',
  )
  declare_useBoundary = DeclareLaunchArgument(
      'useBoundary',
      default_value='false',
      description='Use boundary for navigation',
  )
  declare_num_robots = DeclareLaunchArgument(
      'numRobots',
      default_value='1',
      description='Number of robots',
  )

  return LaunchDescription([
      declare_configFile,
      declare_rviz,
      declare_useBoundary,
      declare_namespace,
      declare_robotName,
      declare_num_robots,
      OpaqueFunction(function=launch_tare_node),
  ])
