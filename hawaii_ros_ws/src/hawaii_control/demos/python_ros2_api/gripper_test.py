from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import time

def main():
    bot = InterbotixManipulatorXS(
        robot_model='Student_Arm',
        group_name='arm',
        gripper_name='gripper'
    )

    # bot.gripper.set_gripper_position(-3.5)
    # bot.gripper.set_gripper_position(-4.0)
    # print(bot.gripper.core.robot_get_motor_registers(cmd_type='single', name="gripper", reg="Present_Load"))
    bot.gripper.set_gripper_position(-2.0)
    time.sleep(1.5)
    # print(bot.gripper.core.robot_get_motor_registers(cmd_type="single", name="gripper", reg="Present_Load"))
    bot.gripper.set_gripper_position(3.141592)
    # time.sleep(0.5)
    # print(bot.gripper.core.robot_get_motor_registers(cmd_type="single", name="gripper", reg="Present_Load"))
    # bot.gripper.set_gripper_position(4.54)
   
    bot.shutdown()


if __name__ == '__main__':
    main()
