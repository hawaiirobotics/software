import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hawaii_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    scripts=[
        'demos/python_ros2_api/bartender.py',
        'demos/python_ros2_api/ee_cartesian_trajectory.py',
        'demos/python_ros2_api/ee_pose_components.py',
        'demos/python_ros2_api/ee_pose_matrix_control.py',
        'demos/python_ros2_api/gripper_control.py',
        'demos/python_ros2_api/joint_current_control.py',
        'demos/python_ros2_api/joint_position_control.py',
        'demos/python_ros2_api/joint_pwm_control.py',
        'demos/python_ros2_api/joint_trajectory_control.py',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hawaii',
    maintainer_email='ramseyer.jessie@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
