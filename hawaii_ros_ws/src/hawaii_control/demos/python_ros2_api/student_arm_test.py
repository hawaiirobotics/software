#!/usr/bin/env python3

import threading

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np

def operateBotR(botR):
    botR.arm.set_single_joint_position(joint_name='Joint1', position=np.pi/2, moving_time= 5, accel_time = 0.5, blocking=False)
    # botR.arm.set_single_joint_position(joint_name='Joint2', position=0.728)
    # botR.arm.set_single_joint_position(joint_name='Joint4', position=np.pi/2.0)
    # botR.arm.set_single_joint_position(joint_name='Joint5', position=0.728)
    # botR.gripper.release()
    # botR.gripper.grasp()
    # botR.gripper.release()
    # botR.gripper.grasp()
    # botR.arm.set_single_joint_position(joint_name='Joint1', position=-np.pi/2.0)
    # botR.arm.set_single_joint_position(joint_name='Joint3', position=-0.528)
    # botR.arm.go_to_home_pose()
   

def operateBotL(botL):
    botL.arm.set_single_joint_position(joint_name='Joint1', position=np.pi/2.0)
    botL.arm.set_single_joint_position(joint_name='Joint2', position=0.728)
    botL.arm.set_single_joint_position(joint_name='Joint4', position=np.pi/2.0)
    botL.arm.set_single_joint_position(joint_name='Joint5', position=0.728)
    botL.gripper.release()
    botL.gripper.grasp()
    botL.gripper.release()
    botL.gripper.grasp()
    botL.arm.set_single_joint_position(joint_name='Joint1', position=-np.pi/2.0)
    botL.arm.set_single_joint_position(joint_name='Joint3', position=-0.528)
    botL.arm.go_to_home_pose()
   
def main():
    
    botR = InterbotixManipulatorXS(
        robot_model='Student_Arm',
        group_name='arm',
        gripper_name='gripper',
        robot_name='student_right',
        init_node=True,
    )
    botL = InterbotixManipulatorXS(
        robot_model='Student_Arm',
        group_name='arm',
        gripper_name='gripper',
        robot_name='student_left',
        init_node=False, #InterbotixManipulatorXS initializes rclpy, which can only be done once
        # set this to false to prevent initializing again
    )

    # operateBotR(botR)
    # botR.arm.set_single_joint_position(joint_name='Joint1', position=np.pi/2, moving_time= 5, accel_time = 0.5, blocking=False)
    # botL.arm.set_single_joint_position(joint_name='Joint1', position=np.pi/2, moving_time= 5, accel_time = 0.5, blocking=False)
    botR.arm.set_joint_positions_for_homing([-3.0, 0.5, 0.5, 0.5, 0.5, 0.5], blocking=False)

    # threadR = threading.Thread(target=operateBotR, args=(botR,))
    # threadL = threading.Thread(target=operateBotR, args=(botR,))

    # threadR.start()
    # threadL.start()

    # threadR.join()
    # threadL.join()

    botR.shutdown()
    botL.shutdown()


if __name__ == '__main__':
    main()
