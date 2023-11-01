import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    OpaqueFunction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, Command)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    world_filepath_launch_arg = LaunchConfiguration('world_filepath')
    use_gazebo_gui_launch_arg = LaunchConfiguration('use_gazebo_gui')
    verbose_launch_arg = LaunchConfiguration('verbose')
    debug_launch_arg = LaunchConfiguration('debug')
    paused_launch_arg = LaunchConfiguration('paused')
    recording_launch_arg = LaunchConfiguration('recording')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')


    # Set ignition resource path
    gz_resource_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            '/usr/share/gazebo-11/models/',
            ':',
            str(Path(
                FindPackageShare('hawaii_descriptions').perform(context)
            ).parent.resolve()),
        ]
    )

    # Set GAZEBO_MODEL_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri_env_var = SetEnvironmentVariable(name='GAZEBO_MODEL_URI',value=[''])

    # Set GAZEBO_MODEL_DATABASE_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri_env_var = SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI',value=[''])

    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ]),
        ]),
        launch_arguments={
            'verbose': verbose_launch_arg,
            'world': world_filepath_launch_arg,
            'pause': paused_launch_arg,
            'record': recording_launch_arg,
            'gdb': debug_launch_arg,
            'valgrind': debug_launch_arg,
            'gui': use_gazebo_gui_launch_arg,
        }.items(),
    )

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_{robot_name_launch_arg.perform(context)}',
        arguments=[
            '-entity', 'robot_description',
            '-topic', f'{robot_name_launch_arg.perform(context)}/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
        output={'both': 'log'},
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            'joint_state_broadcaster',
            '-c',
            '/controller_manager',
        ],
        parameters=[{'use_sim_time': use_sim_time,}],
    )

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            'arm_controller',
            '-c',
            '/controller_manager',
        ],
        parameters=[{'use_sim_time': use_sim_time, }]
    )

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            'gripper_controller',
            '-c',
            '/controller_manager',
        ],
        parameters=[{'use_sim_time': use_sim_time,}]
    )

    hawaii_descriptions_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('hawaii_descriptions'),
                'launch',
                'hawaii_descriptions.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[spawn_joint_state_broadcaster_node]
        )
    )

    load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_arm_controller_node]
        )
    )

    load_gripper_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_gripper_controller_node]
        )
    )

    return [
        gz_resource_path_env_var,
        gz_model_uri_env_var,
        gazebo_launch_include,
        spawn_robot_node,
        load_joint_state_broadcaster_event,
        load_arm_controller_event,
        load_gripper_controller_event,
        hawaii_descriptions_launch_include,
    ]


def generate_launch_description():

    # Set the path to this package.
    pkg_share = FindPackageShare(package='hawaii_gazebo').find('hawaii_gazebo')
    # Set the path to the RViz configuration settings
    default_rviz_config_path = os.path.join(pkg_share, 'worlds/hawaii_gazebo.world')

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'))
    declared_arguments.append(DeclareLaunchArgument('robot_model', default_value='hawaii'))
    declared_arguments.append(DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model')))
    declared_arguments.append(DeclareLaunchArgument('world_filepath', default_value=default_rviz_config_path, description="the file path to the Gazebo 'world' file to load."))
    declared_arguments.append(DeclareLaunchArgument('use_gazebo_gui',default_value='true',choices=('true', 'false')))
    declared_arguments.append(DeclareLaunchArgument('use_rviz', default_value='false'))
    declared_arguments.append(DeclareLaunchArgument('hardware_type', default_value='gz_classic'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            choices=('true', 'false'),
            description='launches Gazebo with verbose console logging if `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            choices=('true', 'false'),
            description='start gzserver in debug mode using gdb.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            choices=('true', 'false'),
            description='start Gazebo in a paused state.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'recording',
            default_value='false',
            choices=('true', 'false'),
            description='enable Gazebo state log recording.',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
