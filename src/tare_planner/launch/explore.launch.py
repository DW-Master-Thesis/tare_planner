from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_tare_node(context, worldName, robotName):
    world_name_str = str(worldName.perform(context))
    robot_name_str = str(robotName.perform(context))
    namespace = "robot_" + robot_name_str
    tare_planner_node = Node(
        package='tare_planner',
        executable='tare_planner_node',
        name='tare_planner_node',
        output='screen',
        parameters=[
          get_package_share_directory('tare_planner') + "/" + world_name_str + '.yaml',
        {"kRobotId": int(robot_name_str)},
        ],
        namespace=namespace,
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='tare_planner_ground_rviz',
        arguments=[
            '-d', get_package_share_directory('tare_planner')+'/tare_planner_ground.rviz'],
        namespace=namespace,
    )
    return [rviz_node, tare_planner_node]


def generate_launch_description():

    useSimTime = LaunchConfiguration('useSimTime')
    robotName = LaunchConfiguration('robotName')
    worldName = LaunchConfiguration('worldName')
    rviz = LaunchConfiguration('rviz')
    useBoundary = LaunchConfiguration('useBoundary')

    declare_useSimTime = DeclareLaunchArgument(
        'useSimTime',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_worldName = DeclareLaunchArgument(
        'worldName',
        default_value='indoor',
        description='',
    )
    declare_robotName = DeclareLaunchArgument(
        'robotName',
        default_value='0',
        description='',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='',
    )
    declare_useBoundary = DeclareLaunchArgument(
        'useBoundary',
        default_value='false',
        description='Use boundary for navigation',
    )

    return LaunchDescription([
        declare_useSimTime,
        declare_worldName,
        declare_rviz,
        declare_useBoundary,
        declare_robotName,
        OpaqueFunction(function=launch_tare_node, args=[worldName, robotName])
    ])
