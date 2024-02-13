from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from interbotix_xs_modules.xs_launch.xs_launch import determine_use_sim_time_param
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
    rvizconfig_launch_arg = LaunchConfiguration('rvizconfig')
    using_one_arm = LaunchConfiguration('one_arm').perform(context)

    # sets use_sim_time parameter to 'true' if using gazebo hardware
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        hardware_type_launch_arg=hardware_type_launch_arg
    )

    xsarm_description_launch_include = IncludeLaunchDescription(
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
            'robot_description': robot_description_launch_arg,
            'use_sim_time': use_sim_time_param,
            'use_joint_pub_gui' : 'false',
            'rvizconfig' : rvizconfig_launch_arg,
            'one_arm' : using_one_arm,
        }.items(),
    )

    namespace = None
    if using_one_arm == 'true':
        namespace = robot_name_launch_arg

    xs_sdk_node = Node(
        condition=UnlessCondition(use_sim_launch_arg),
        package='interbotix_xs_sdk',
        executable='xs_sdk',
        name='xs_sdk',
        namespace=namespace,
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
        namespace=namespace,
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
        xsarm_description_launch_include,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            default_value='Student_Arm'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                'name of the robot (typically equal to `robot_model`, but could be anything).'
            ),
        )
    )
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
            default_value='DEBUG',
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
            default_value='false',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock; this value is automatically set to `true` if'
                ' using Gazebo hardware.'
            )
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('hawaii_descriptions'),
                'rviz',
                'hawaii.rviz',
            ]),
            description='file path to the config file RViz should load.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'one_arm',
            default_value='true',
            description='Only launching one arm in total.',
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments()
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
