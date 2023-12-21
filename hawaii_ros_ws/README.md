# Usage:

**On first use:**

`export ROS_DISTRO=humble`

`rosdep install --from-paths ~/software/hawaii_ros_ws/src --ignore-src -r -y`

This will install all package dependencies for you.

**Run in hawaii_ros_ws directory:**

1. Build packages:

`colcon build`

2. Source ros workspace:

`source install/setup.bash`

3. Run launch file:

    ***To launch Rviz simulation on its own:***
    
    `ros2 launch hawaii_descriptions hawaii_descriptions.launch.py`
    
    ***To control a single arm:***
    
    `ros2 launch hawaii_control hawaii_control.launch.py use_sim:=<true or false>`
    
    true = run in simulation
    false =  run on hardware
  
    ***To launch 2 arms for teleop***
    
    `bash hawaii_teleop.bash`

# Teleop Software Architecture

![hg](https://github.com/hawaiirobotics/software/assets/54551825/62a861b0-3dd4-4b38-8217-f54dcffa96fe)

