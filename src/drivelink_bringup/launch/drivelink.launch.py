from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    uart_port = LaunchConfiguration('uart_port')
    uart_baud = LaunchConfiguration('uart_baud')
    telemetry_period_ms = LaunchConfiguration('telemetry_period_ms')
    imu_stream_period_ms = LaunchConfiguration('imu_stream_period_ms')

    bringup_share = FindPackageShare('drivelink_bringup')
    xacro_file = PathJoinSubstitution([bringup_share, 'urdf', 'robot.urdf.xacro'])
    controllers_file = PathJoinSubstitution([bringup_share, 'config', 'controllers.yaml'])

    robot_description = Command([
        'xacro ', xacro_file,
        ' uart_port:=', uart_port,
        ' uart_baud:=', uart_baud,
        ' telemetry_period_ms:=', telemetry_period_ms,
        ' imu_stream_period_ms:=', imu_stream_period_ms,
    ])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers_file],
        output='screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    imu_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    velocity_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    effort_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_effort_controller', '--inactive', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[imu_spawner, velocity_spawner, effort_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('uart_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('uart_baud', default_value='115200'),
        DeclareLaunchArgument('telemetry_period_ms', default_value='100'),
        DeclareLaunchArgument('imu_stream_period_ms', default_value='10'),
        rsp,
        control_node,
        joint_state_spawner,
        spawn_after_joint_state,
    ])
