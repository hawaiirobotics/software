from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path

def generate_launch_description():

    robot_model = LaunchConfiguration('robot_model')
    student_left_name = LaunchConfiguration('student_left_name')
    student_right_name = LaunchConfiguration('student_right_name')
    student_left_mode = LaunchConfiguration('student_modes_left')
    student_right_mode = LaunchConfiguration('student_modes_right')
    use_sim = LaunchConfiguration('use_sim')

    x_spawn_left = '2'
    y_spawn_left = '0'
    x_spawn_right = '-2'
    y_spawn_right = '0'


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

        SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            '/usr/share/gazebo-11/models/',
            ':',
            str(PathJoinSubstitution([FindPackageShare('hawaii_descriptions')]))]
        ),

        # Set GAZEBO_MODEL_URI to empty string to prevent Gazebo from downloading models
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_URI',
            value=['']
        ),

        # Set GAZEBO_MODEL_DATABASE_URI to empty string to prevent Gazebo from downloading models
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_DATABASE_URI',
            value=['']
        ),

        ExecuteProcess(cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_factory.so'], output='screen'),

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
                'base_link_frame': "base_link",
                'use_world_frame': 'true',
                'use_rviz': 'false',
                'mode_configs': student_right_mode,
                'use_sim': use_sim,
                'x_spawn' : x_spawn_right,
                'y_spawn' : y_spawn_right,
            }.items()
        ),
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
                'base_link_frame': "base_link",
                'use_world_frame': 'true',
                'use_rviz': 'true',
                'mode_configs': student_left_mode,
                'use_sim': use_sim,
                'x_spawn' : x_spawn_left,
                'y_spawn' : y_spawn_left,
            }.items()
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='student_left_transform_broadcaster',
            output='screen',
            arguments=[x_spawn_left, y_spawn_left, '0', '0', '0', '0', '/world', f"/student_left/base_link"] #update x,y,z,and quats
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='student_right_transform_broadcaster',
            output='screen',
            arguments=[x_spawn_right, y_spawn_right, '0', '0', '0', '0', '/world', f"/student_right/base_link"] #update x,y,z,and quats
        ),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_1',
            output='screen',
            parameters=[
                {'video_device': '/dev/CAM_1'},
                {'framerate': 60.0},
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'},
                {'camera_frame_id': 'usb_cam'},
                {'io_method': 'mmap'},
                {'autofocus': False},
                {'focus': 5},
                {'autoexposure': True}
            ]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_2',
            output='screen',
            parameters=[
                {'video_device': '/dev/CAM_2'},
                {'framerate': 60.0},
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'},
                {'camera_frame_id': 'usb_cam'},
                {'io_method': 'mmap'},
                {'autofocus': False},
                {'focus': 5},
                {'autoexposure': True}
            ]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_3',
            output='screen',
            parameters=[
                {'video_device': '/dev/CAM_3'},
                {'framerate': 60.0},
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'},
                {'camera_frame_id': 'usb_cam'},
                {'io_method': 'mmap'},
                {'autofocus': False},
                {'focus': 5},
                {'autoexposure': True}
            ]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_4',
            output='screen',
            parameters=[
                {'video_device': '/dev/CAM_4'},
                {'framerate': 60.0},
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'},
                {'camera_frame_id': 'usb_cam'},
                {'io_method': 'mmap'},
                {'autofocus': False},
                {'focus': 5},
                {'autoexposure': True}
            ]
        )
    ])
