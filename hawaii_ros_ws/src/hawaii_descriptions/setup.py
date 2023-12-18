import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'hawaii_descriptions'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
  	(os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
  	(os.path.join('share', package_name, 'meshes'), glob('meshes/*.png')),
  	(os.path.join('share', package_name, 'meshes/hawaii_meshes'), glob('meshes/hawaii_meshes/*')),
  	(os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jramseyer',
    maintainer_email='ramseyer.jessie@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'hawaii_descriptions = hawaii_descriptions.hawaii_descriptions:main'
        ],
    },
)
