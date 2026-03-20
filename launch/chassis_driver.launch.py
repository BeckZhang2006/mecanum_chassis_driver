from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包的share目录路径
    pkg_share = FindPackageShare('mecanum_chassis_driver')

    # 默认配置文件路径
    default_config_path = PathJoinSubstitution([
        pkg_share, 'config', 'chassis_config.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='串口设备路径'),

        DeclareLaunchArgument(
            'config',
            default_value=default_config_path,
            description='配置文件路径（YAML格式）'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'),

        Node(
            package='mecanum_chassis_driver',
            executable='chassis_driver',
            name='mecanum_chassis_driver',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),  # 加载配置文件
                {'port': LaunchConfiguration('port')},  # 命令行参数覆盖
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('cmd_vel', 'cmd_vel'),
                ('odom', 'odom'),
            ]
        ),
    ])
