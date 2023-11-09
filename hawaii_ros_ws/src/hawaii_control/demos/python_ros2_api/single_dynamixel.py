import sys

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np

def main():
    
    bot = InterbotixManipulatorXS(
        robot_model='hawaii',
        group_name='arm',
        gripper_name=None,
    )

    bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/2.0)
    bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi/2.0)
    bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/2.0)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

    bot.shutdown()


if __name__ == '__main__':
    main()