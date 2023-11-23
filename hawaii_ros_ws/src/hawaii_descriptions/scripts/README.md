# Kinematics_from_description.py

The `kinematics_from_description.py` script computes necessary properties for running portions of the Modern Robotics Library using a robot's URDF. Specifically, this program calculates the `M` and `Slist` parameters required by MR kinematics functions:

- The `M` matrix represents the end effector configuration in SE(3) when the robot is at its home position (i.e., all joints are at position 0).
- The `Slist` matrix denotes the joint screw axes expressed in the space frame when the robot is at its home position. Each axis is formatted as a column within this matrix.

This script utilizes the `urdfpy` Python package, which mandates that <ins>**the links in the URDF are relative or absolute links, rather than ROS resource URLs**</ins>.

## Usage

1. Install the necessary requirements:
   
   ```bash
   python3 -m pip3 install -r requirements.txt

2. Open kinematics_from_description.py and modify the configurations for:

    `space_frame`: The name assigned to the link representing the location of the robot's base or {0} link.
    
    `body_frame`: The name assigned to the link indicating the location of the end effector.
3. Update the file path to point to your URDF file.

4. Copy the output `M` and `Slist` Matrices to the `interbotix_xs_modules/xs_robot/mr_descriptions.py` script for your robot.

    This script facilitates the extraction of essential kinematic parameters from your robot's URDF, preparing them for use within the Modern Robotics Library functions.
