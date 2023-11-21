The kinematics_from_description.py script calculates the properties required to run parts of the Modern Robotics Library from a robot's URDF. Specifically, this program calculates the M and Slist parameters required by MR kinematics functions:

    The M matrix is the end effector configuration in SE(3) when the robot is in its home position (i.e. all joints are at position 0).
    The Slist matrix is the joint screw axes expressed in the space frame when the robot is in its home position. This matrix is formatted to have each axis as a column
    
    This script uses the urdfpy python package which requires that the links in the urdf are relative or absolute links rather than ROS resource URLs.
    
    To run the script:
    
    run python3 -m pip3 install -r requirements.txt
    
    open kinematics_from_description.py and change the body_frame and space_frame configs to what you need them to be.
    
space_frame - The name given to the link that will serve as the location of the robot's base, or {0} link.
body_frame - The name given to the link that will serve as the location of the end effector.

also change the file path to the path to your urdf file

Copy the output M and Slist Matrices to the interbotix_xs_modules/xs_robot/mr_descriptions.py script for your robot.

