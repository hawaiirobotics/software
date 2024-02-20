#!/usr/bin/env python3

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import time
   
def main():
    
    bot = InterbotixManipulatorXS(
        robot_model='Student_Arm',
        group_name='arm',
        gripper_name='gripper',
        init_node=True,
    )
    # bot.arm.set_joint_positions([joint_name='Joint5'], position=np.pi/2.0, moving_time=10, accel_time=0.5)
    # bot.arm.go_to_sleep_pose(moving_time=10, accel_time=0.5)
    bot.arm.set_single_joint_position(joint_name='Joint4', position=-0.5)
    # time.sleep(0.5)
    # bot.arm.set_single_joint_position(joint_name='Joint2', position=np.pi/2.0, moving_time=2, accel_time=0.5)
    # bot.arm.set_single_joint_position(joint_name='Joint2', position=0.728)
    # bot.arm.set_single_joint_position(joint_name='Joint4', position=np.pi/2.0)
    # bot.arm.set_single_joint_position(joint_name='Joint5', position=0.728)
    # # bot.gripper.release()
    # # bot.gripper.grasp()
    # # bot.gripper.release()
    # # bot.gripper.grasp()
    # bot.arm.set_single_joint_position(joint_name='Joint1', position=-np.pi/2.0)
    # bot.arm.set_single_joint_position(joint_name='Joint3', position=-0.528)
    # bot.arm.go_to_home_pose()

    bot.shutdown()


if __name__ == '__main__':
    main()

