#!/usr/bin/env bash

# Source the workspace
source ./install/setup.bash

# Run your launch file
ros2 launch hawaii_teleop hawaii_teleop.launch.py $@ &
ROS_NODES_PIDS=($!)

# Wait for the launch file to start up (adjust sleep time as needed)
sleep 5

X_SPAWN_LEFT=1
X_SPAWN_RIGHT=-1
YAW_SPAWN_LEFT=-1.57075
YAW_SPAWN_RIGHT=1.57075

# Run static_transform_publisher instances
# ARGS = [x y z yaw pitch roll frame_id child_frame_id]
ros2 run tf2_ros static_transform_publisher $X_SPAWN_LEFT 0 0 $YAW_SPAWN_LEFT 0 0 /world /student_left/base_link &
ros2 run tf2_ros static_transform_publisher $X_SPAWN_RIGHT 0 0 $YAW_SPAWN_RIGHT 0 0 /world /student_right/base_link &

v4l2-ctl --device=/dev/CAM_HIGH --set-ctrl=auto_exposure=1
v4l2-ctl --device=/dev/CAM_FRONT --set-ctrl=auto_exposure=1
v4l2-ctl --device=/dev/CAM_LEFT --set-ctrl=auto_exposure=1
v4l2-ctl --device=/dev/CAM_RIGHT --set-ctrl=auto_exposure=1

# Store the PIDs of the static_transform_publisher instances
STATIC_TRANSFORM_PIDS=($!)

# Wait for all background processes to finish
wait "${ROS_NODES_PIDS[@]}" "${STATIC_TRANSFORM_PIDS[@]}"

