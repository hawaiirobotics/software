from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

def main():
    bot = InterbotixManipulatorXS(
        robot_model='Student_Arm',
        group_name='arm',
        gripper_name='gripper'
    )

    # bot.gripper.set_gripper_position(-3.5)
    # bot.gripper.set_gripper_position(-4.0)
    bot.gripper.set_gripper_position(-2.5)
    # bot.gripper.set_gripper_position(3.141592)
    # bot.gripper.set_gripper_position(4.54)
   
    bot.shutdown()


if __name__ == '__main__':
    main()
