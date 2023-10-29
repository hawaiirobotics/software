import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hawaii_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jramseyer',
    maintainer_email='ramseyer.jessie@gmail.com',
    description='hawaii gazebo simulations',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hawaii_gazebo = hawaii_gazebo.hawaii_gazebo:main'
        ],
    },
)
