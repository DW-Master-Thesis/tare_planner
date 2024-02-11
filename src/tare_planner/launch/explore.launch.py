from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_tare_node(context, worldName, robotName):
    world_name_str = str(worldName.perform(context))
    tare_planner_node = Node(
        package='tare_planner',
        executable='tare_planner_node',
        name='tare_planner_node',
        output='screen',
        parameters=[get_package_share_directory('tare_planner') + "/" + world_name_str + '.yaml'],
        namespace=robotName,
    )
    return [tare_planner_node]


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
        default_value='robot_1',
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

    navigation_boundary_node = Node(
        package='tare_planner',
        executable='navigationBoundary',
        name='navigationBoundary',
        output='screen',
        parameters=[
            {'boundary_file_dir': get_package_share_directory('tare_planner') + '/data/boundary.ply'},
            {'sendBoundary': True},
            {'sendBoundaryInterval': 2}
        ],
        condition=IfCondition(useBoundary),
        namespace=robotName,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='tare_planner_ground_rviz',
        arguments=[
            '-d', get_package_share_directory('tare_planner')+'/tare_planner_ground.rviz'],
        condition=IfCondition(rviz),
        namespace=robotName,
    )

    return LaunchDescription([
        declare_useSimTime,
        declare_worldName,
        declare_rviz,
        declare_useBoundary,
        declare_robotName,
        navigation_boundary_node,
        rviz_node,
        OpaqueFunction(function=launch_tare_node, args=[worldName, robotName])
    ])
