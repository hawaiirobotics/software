#!/usr/bin/env python3

import sys

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np

def main():
    
    bot = InterbotixManipulatorXS(
        robot_model='Student_Arm',
        group_name='arm',
        gripper_name='gripper',
        robot_name='student_right'
    )

    bot.arm.go_to_sleep_pose()
    #bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    #bot.arm.set_single_joint_position(joint_name='J1', position=np.pi/2.0)
    #bot.arm.set_single_joint_position(joint_name='J2', position=0.728)
    bot.gripper.release()

    #bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    bot.gripper.grasp()
    #bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    # bot.arm.set_single_joint_position(joint_name='J1', position=-np.pi/2.0)
    # bot.arm.set_single_joint_position(joint_name='J3', position=-0.528)
    #bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    #bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    # bot.arm.set_single_joint_position(joint_name='J1', position=np.pi/2.0)
    #bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    bot.gripper.release()
    #bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

    bot.shutdown()


if __name__ == '__main__':
    main()
