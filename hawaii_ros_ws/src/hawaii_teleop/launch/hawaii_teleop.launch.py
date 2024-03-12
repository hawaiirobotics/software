from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    GroupAction,
    ExecuteProcess
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from scripts import GazeboRosPaths

def generate_launch_description():

    robot_model = LaunchConfiguration('robot_model')
    student_left_name = LaunchConfiguration('student_left_name')
    student_right_name = LaunchConfiguration('student_right_name')
    student_left_mode = LaunchConfiguration('student_modes_left')
    student_right_mode = LaunchConfiguration('student_modes_right')
    use_sim = LaunchConfiguration('use_sim')

    # x_spawn_left = '1'
    # y_spawn_left = '0'
    # yaw_spawn_left = '-1.57075'
    # x_spawn_right = '-1'
    # y_spawn_right = '0'
    # yaw_spawn_right = '1.57075'

    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    env = {
        "GAZEBO_MODEL_PATH": model_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "GAZEBO_RESOURCE_PATH": media_path,
    }

    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='Student_Arm'),
        DeclareLaunchArgument('student_left_name', default_value='student_left'),
        DeclareLaunchArgument('student_right_name', default_value='student_right'),
        DeclareLaunchArgument('student_modes_left', default_value=PathJoinSubstitution([
                                                    FindPackageShare('hawaii_teleop'),
                                                    'config',
                                                    'student_modes_left.yaml',
                                                ])),
        DeclareLaunchArgument('student_modes_right', default_value=PathJoinSubstitution([
                                                    FindPackageShare('hawaii_teleop'),
                                                    'config',
                                                    'student_modes_right.yaml',
                                                ])),
        DeclareLaunchArgument('use_sim', default_value='true'),
        
        GroupAction([
            # Instances use the robot's name for namespace
            PushRosNamespace(student_right_name),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('hawaii_control'),
                        'launch',
                        'hawaii_control.launch.py'])
                ]),
                launch_arguments={
                    'robot_model': robot_model,
                    'robot_name': student_right_name,
                    'base_link_frame': "/base_link",
                    'use_world_frame': 'true',
                    'use_rviz': 'true',
                    'rvizconfig': 
                    PathJoinSubstitution([
                        FindPackageShare('hawaii_descriptions'),
                        'rviz',
                        'hawaii_two_arm_teleop.rviz',
                    ]),
                    'mode_configs' : student_right_mode,
                    'use_sim' : use_sim,
                    'one_arm' : 'false',
                    # 'x_spawn' : x_spawn_right,
                    # 'y_spawn' : y_spawn_right,
                    # 'yaw_spawn' : yaw_spawn_right,
                }.items()
            ),
        ]),

        GroupAction([
            # Instances use the robot's name for namespace
            PushRosNamespace(student_left_name),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('hawaii_control'),
                        'launch',
                        'hawaii_control.launch.py'])
                    ]),
                launch_arguments={
                    'robot_model': robot_model,
                    'robot_name': student_left_name,
                    'base_link_frame': "/base_link",
                    'use_world_frame': 'true',
                    'use_rviz': 'false',
                    'mode_configs': student_left_mode,
                    'use_sim' : use_sim,
                    'one_arm' : 'false',
                    # 'x_spawn' : x_spawn_left,
                    # 'y_spawn' : y_spawn_left,
                    # 'yaw_spawn' : yaw_spawn_left,
                }.items()
            ),
        ]),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_high',
            output='screen',
            namespace='usb_cam_high',
            parameters=[
                {'video_device': '/dev/CAM_HIGH'},
                {'framerate': 60.0},
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'},
                {'camera_frame_id': 'usb_cam'},
                {'io_method': 'mmap'},
                {'autofocus': False},
                {'focus': 5},
                # {'autoexposure': False}
            ]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_front',
            output='screen',
            namespace='usb_cam_front',
            parameters=[
                {'video_device': '/dev/CAM_FRONT'},
                {'framerate': 60.0},
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'},
                {'camera_frame_id': 'usb_cam'},
                {'io_method': 'mmap'},
                {'autofocus': False},
                {'focus': 5},
                # {'autoexposure': False}
            ]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_left',
            output='screen',
            namespace='usb_cam_left',
            parameters=[
                {'video_device': '/dev/CAM_LEFT'},
                {'framerate': 60.0},
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'},
                {'camera_frame_id': 'usb_cam'},
                {'io_method': 'mmap'},
                {'autofocus': False},
                {'focus': 5},
                # {'autoexposure': False}
            ]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_right',
            output='screen',
            namespace='usb_cam_right',
            parameters=[
                {'video_device': '/dev/CAM_RIGHT'},
                {'framerate': 60.0},
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'},
                {'camera_frame_id': 'usb_cam'},
                {'io_method': 'mmap'},
                {'autofocus': False},
                {'focus': 5},
                # {'autoexposure': False}
            ]
        )
    ])
