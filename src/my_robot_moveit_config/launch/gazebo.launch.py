import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_my_robot_moveit_config = FindPackageShare('my_robot_moveit_config')

    # Paths
    gz_sim_launch_file = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    urdf_file = PathJoinSubstitution([pkg_my_robot_moveit_config, 'config', 'my_robot.urdf.xacro'])
    
    # Process URDF with xacro
    robot_description_content = Command([
        'xacro ', urdf_file
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # Launch Gazebo (gz-sim) with simulation time
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_file),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Clock bridge - CRITICAL for sim_time synchronization
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Robot State Publisher with sim_time
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Spawn Joint State Broadcaster (delayed 20 seconds)
    spawn_joint_state_broadcaster = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    # Spawn Arm Controller (delayed 25 seconds)
    spawn_arm_controller = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '-c', '/controller_manager'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    # Spawn Gripper Controller (delayed 30 seconds)
    spawn_gripper_controller = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller', '-c', '/controller_manager'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_entity,
        spawn_joint_state_broadcaster,
        spawn_arm_controller,
        spawn_gripper_controller,
    ])
