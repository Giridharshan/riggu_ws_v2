from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    uart_port = LaunchConfiguration('uart_port')
    uart_baud = LaunchConfiguration('uart_baud')
    imu_stream_period_ms = LaunchConfiguration('imu_stream_period_ms')
    bringup_share = FindPackageShare('drivelink_bringup')

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([bringup_share, 'urdf', 'imu.urdf.xacro']),
        ' uart_port:=', uart_port,
        ' uart_baud:=', uart_baud,
        ' telemetry_period_ms:=100',
        ' imu_stream_period_ms:=', imu_stream_period_ms,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('uart_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('uart_baud', default_value='115200'),
        DeclareLaunchArgument('imu_stream_period_ms', default_value='10'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                PathJoinSubstitution([bringup_share, 'config', 'controllers.yaml']),
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['imu_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
    ])
