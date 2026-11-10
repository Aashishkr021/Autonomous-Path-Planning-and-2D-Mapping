import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))
CFG = os.path.join(ROOT, 'config')


def generate_launch_description():
    slam_launch = os.path.join(get_package_share_directory('slam_toolbox'),
                               'launch', 'online_async_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true',
                              description='true: slam_toolbox map. false: simulation-only grid_mapper (exact odometry, no SLAM)'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(CFG, 'bridge.yaml')}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '0', '--y', '0', '--z', '0.1',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={'slam_params_file': os.path.join(CFG, 'slam.yaml'),
                              'use_sim_time': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('slam'))),
        ExecuteProcess(cmd=['python3', os.path.join(ROOT, 'scripts', 'grid_mapper.py'),
                            '--ros-args', '-p', 'use_sim_time:=true'],
                       output='screen', condition=UnlessCondition(LaunchConfiguration('slam'))),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(ROOT, 'rviz', 'airmouse.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
