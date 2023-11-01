from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    motor_configs_launch_arg = LaunchConfiguration('motor_configs')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    load_configs_launch_arg = LaunchConfiguration('load_configs')
    use_sim_launch_arg = LaunchConfiguration('use_sim')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')
    use_sim_time_param = LaunchConfiguration('use_sim_time')

    hawaii_description_launch_include = IncludeLaunchDescription(
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
            'use_sim_time': use_sim_time_param,
        }.items(),
    )

    xs_sdk_node = Node(
        condition=UnlessCondition(use_sim_launch_arg),
        package='interbotix_xs_sdk',
        executable='xs_sdk',
        name='xs_sdk',
        namespace=robot_name_launch_arg,
        arguments=[],
        parameters=[{
            'motor_configs': motor_configs_launch_arg,
            'mode_configs': mode_configs_launch_arg,
            'load_configs': load_configs_launch_arg,
            'robot_description': robot_description_launch_arg,
            'use_sim_time': use_sim_time_param,
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }],
        output={'both': 'screen'},
    )

    xs_sdk_sim_node = Node(
        condition=IfCondition(use_sim_launch_arg),
        package='interbotix_xs_sdk',
        executable='xs_sdk_sim.py',
        name='xs_sdk_sim',
        namespace=robot_name_launch_arg,
        arguments=[],
        parameters=[{
            'motor_configs': motor_configs_launch_arg,
            'mode_configs': mode_configs_launch_arg,
            'robot_description': robot_description_launch_arg,
            'use_sim_time': use_sim_time_param,
        }],
        output={'both': 'screen'},
    )

    return [
        xs_sdk_node,
        xs_sdk_sim_node,
        hawaii_description_launch_include,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('robot_model', default_value='hawaii'))
    declared_arguments.append(DeclareLaunchArgument('robot_name', default_value=LaunchConfiguration('robot_model')))
    declared_arguments.append(DeclareLaunchArgument(
        'robot_description', default_value=Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('hawaii_descriptions'),
                'urdf',
                LaunchConfiguration('robot_model')
            ]), '.urdf.xacro '])))
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'motor_configs',
            default_value=[PathJoinSubstitution([
                FindPackageShare('hawaii_control'),
                'config',
                LaunchConfiguration('robot_model')]),
                '.yaml'
            ],
            description="the file path to the 'motor config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('hawaii_control'),
                'config',
                'modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'load_configs',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'a boolean that specifies whether or not the initial register values '
                "(under the 'motors' heading) in a Motor Config file should be written "
                "to the motors; as the values being written are stored in each motor's "
                'EEPROM (which means the values are retained even after a power cycle), '
                'this can be set to `false` after the first time using the robot. Setting '
                'to `false` also shortens the node startup time by a few seconds and '
                'preserves the life of the EEPROM.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'xs_driver_logging_level',
            default_value='INFO',
            choices=('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'),
            description='set the logging level of the X-Series Driver.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'if `true`, the DYNAMIXEL simulator node is run; use RViz to visualize the'
                " robot's motion; if `false`, the real DYNAMIXEL driver node is run."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock; this value is automatically set to `true` if'
                ' using Gazebo hardware.'
            )
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
