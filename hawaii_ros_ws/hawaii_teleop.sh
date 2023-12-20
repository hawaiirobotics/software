#!/bin/bash

# Run your launch file
ros2 launch hawaii_teleop hawaii_teleop.launch.py &

# Wait for the launch file to start up (adjust sleep time as needed)
sleep 5

# Run static_transform_publisher instances
ros2 run tf2_ros static_transform_publisher 1 0 0 -1.57075 0 0 /world /student_left/base_link &
ros2 run tf2_ros static_transform_publisher -1 0 0 1.57075 0 0 /world /student_right/base_link &
