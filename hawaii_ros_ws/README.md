# To launch Rviz simulation:

**On first use:**

`export ROS_DISTRO=humble`

`rosdep install --from-paths ~/software/hawaii_ros_ws/src --ignore-src -r -y`

This will install all package dependencies for you.

**Run in hawaii_ros_ws directory:**

Build packages:

`colcon build`

Source ros workspace:

`source install/setup.bash`

Run launch file (launches Rviz):

`ros2 launch hawaii_descriptions hawaii_descriptions.launch.py`
