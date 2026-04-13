import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_dir = get_package_share_directory('turtlebot3-docker')

    # Launch configuration variables (match navigation.launch.py and slam.launch.py)
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    nav_params_file = LaunchConfiguration('params_file')

    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    configuration_basename = LaunchConfiguration('configuration_basename')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Whether run a SLAM',
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml',
        ),
        description='Full path to map file to load',
    )

    # Keep arg name as 'params_file' to match navigation.launch.py
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    param_file_name = turtlebot3_model + '.yaml'
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro == 'humble':
        nav_default_params = os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            ros_distro,
            param_file_name,
        )
    else:
        nav_default_params = os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            param_file_name,
        )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav_default_params,
        description='Full path to param file to load',
    )

    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    declare_cartographer_config_dir_cmd = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=os.path.join(turtlebot3_cartographer_prefix, 'config'),
        description='Full path to config file to load',
    )

    declare_configuration_basename_cmd = DeclareLaunchArgument(
        'configuration_basename',
        default_value='turtlebot3_lds_2d.lua',
        description='Name of lua file for cartographer',
    )

    declare_resolution_cmd = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of a grid cell in the published occupancy grid',
    )

    declare_publish_period_sec_cmd = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='OccupancyGrid publishing period',
    )

    bringup_cmd_group = GroupAction([
        # Navigation stack (includes RViz)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': nav_params_file,
            }.items(),
        ),

        # SLAM stack (disable RViz here to avoid launching RViz twice)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_dir, 'launch', 'slam.launch.py')),
            condition=IfCondition(slam),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_rviz': 'false',
                'cartographer_config_dir': cartographer_config_dir,
                'configuration_basename': configuration_basename,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec,
            }.items(),
        ),
    ])

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_cartographer_config_dir_cmd)
    ld.add_action(declare_configuration_basename_cmd)
    ld.add_action(declare_resolution_cmd)
    ld.add_action(declare_publish_period_sec_cmd)
    ld.add_action(bringup_cmd_group)

    return ld
