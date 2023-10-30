import os, xacro
from ament_index_python.packages import get_package_share_directory
from typing import Optional
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument)
from launch.substitutions import (LaunchConfiguration, Command)
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Set the path to this package.
    pkg_share = FindPackageShare(package='hawaii_descriptions').find('hawaii_descriptions')
    
    # Set the path to the RViz configuration settings
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/hawaii.rviz')
    
    # Set the path to the URDF file
    default_urdf_model_path = os.path.join(pkg_share, 'urdf/hawaii.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')    

    return LaunchDescription([
    	DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('robot_model', default_value='hawaii'),
        DeclareLaunchArgument('robot_name', default_value='$(arg robot_model)'),
        DeclareLaunchArgument('base_link_frame', default_value='base_link'),
        DeclareLaunchArgument('use_gripper', default_value='true'),
        DeclareLaunchArgument('show_ar_tag', default_value='false'),
        DeclareLaunchArgument('show_gripper_bar', default_value='true'),
        DeclareLaunchArgument('show_gripper_fingers', default_value='true'),
        DeclareLaunchArgument('use_world_frame', default_value='true'),
        DeclareLaunchArgument('external_urdf_loc', default_value=''),
        DeclareLaunchArgument('hardware_type', default_value='actual', choices= ('actual','fake', 'gz_classic')),

        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('load_gazebo_configs', default_value='false'),
        DeclareLaunchArgument('use_joint_pub', default_value='false'),
        DeclareLaunchArgument('use_joint_pub_gui', default_value='true'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path, description='Full path to the RVIZ config file to use'),
        DeclareLaunchArgument('robot_description', default_value=Command(['xacro ', default_urdf_model_path])),

        Node(
            condition=UnlessCondition(LaunchConfiguration('use_joint_pub_gui')),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace='hawaii',
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('use_joint_pub_gui')),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace='hawaii',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace='hawaii',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': LaunchConfiguration('robot_description')}],
            #arguments=[default_urdf_model_path],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),
    ])
