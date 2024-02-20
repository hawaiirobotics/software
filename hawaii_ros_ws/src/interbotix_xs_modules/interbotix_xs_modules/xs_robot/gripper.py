"""
Contains the `InterbotixGripperXS` and `InterbotixGripperXSInterface` classes.
These two classes can be used to control an X-Series standalone gripper using Python.
"""

import sys
from threading import Thread
import time

from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.srv import RobotInfo, RegisterValues
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity


class InterbotixGripperXS:
    """Standalone Module to control an Interbotix Gripper using position control."""

    def __init__(
        self,
        robot_model: str,
        gripper_name: str,
        robot_name: str = None,
        topic_joint_states: str = 'joint_states',
        logging_level: LoggingSeverity = LoggingSeverity.INFO,
        node_name: str = 'robot_manipulation',
        start_on_init: bool = True,
    ) -> None:
        """
        Construct the Standalone Interbotix Gripper Module.

        :param robot_model: (ex. 'student_right')
        :param gripper_name: name of the gripper joint as defined in the 'motor_config' yaml file;
            typically, this is 'gripper'
        :param robot_name: (optional) defaults to value given to 'robot_model'; this can be
            customized to best suit the user's needs
        :param topic_joint_states: (optional) the specifc JointState topic output by the xs_sdk
            node
        :param logging_level: (optional) rclpy logging severity level. Can be DEBUG, INFO, WARN,
            ERROR, or FATAL. defaults to INFO
        :param node_name: (optional) name to give to the core started by this class, defaults to
            'robot_manipulation'
        :param start_on_init: (optional) set to `True` to start running the spin thread after the
            object is built; set to `False` if intending to sub-class this. If set to `False`,
            either call the `start()` method later on, or add the core to an executor in another
            thread.
        :details: note that this module doesn't really have any use case except in controlling just
            the gripper joint on a Hawaii arm.
        """
        self.core = InterbotixRobotXSCore(
            robot_model,
            robot_name,
            topic_joint_states=topic_joint_states,
            logging_level=logging_level,
            node_name=node_name,
        )
        self.gripper = InterbotixGripperXSInterface(
            self.core,
            gripper_name,
        )

        if start_on_init:
            self.start()

    def start(self) -> None:
        """Start a background thread that builds and spins an executor."""
        self._execution_thread = Thread(target=self.run)
        self._execution_thread.start()

    def run(self) -> None:
        """Thread target."""
        self.ex = MultiThreadedExecutor()
        self.ex.add_node(self.core)
        self.ex.spin()

    def shutdown(self) -> None:
        """Destroy the node and shut down all threads and processes."""
        self.core.destroy_node()
        rclpy.shutdown()
        self._execution_thread.join()
        time.sleep(0.5)


class InterbotixGripperXSInterface:
    def __init__(
        self,
        core: InterbotixRobotXSCore,
        gripper_name: str,
    ) -> None:
        """
        Construct the Interbotix Gripper Module.

        :param core: reference to the InterbotixRobotXSCore class cdontaining the internal ROS
            plumbing that drives the Python API
        :param gripper_name: name of the gripper joint as defined in the 'motor_config' yaml file;
            typically, this is 'gripper'
        """
        self.core = core
        self.gripper_name = gripper_name
        self.future_gripper_info = self.core.srv_get_info.call_async(
            RobotInfo.Request(cmd_type='single', name=gripper_name)
        )
        self.gripper_command = JointSingleCommand(name=gripper_name)

        self.moving_time = 0.0
        self.accel_time = 0.0

        while rclpy.ok() and not self.future_gripper_info.done():
            rclpy.spin_until_future_complete(self.core, self.future_gripper_info)
            rclpy.spin_once(self.core)

        self.gripper_info: RobotInfo.Response = self.future_gripper_info.result()
        self.left_finger_index = self.core.js_index_map[self.gripper_info.joint_names[0]]
        self.left_finger_lower_limit = self.gripper_info.joint_lower_limits[0]
        self.left_finger_upper_limit = self.gripper_info.joint_upper_limits[0]

        if self.gripper_info.mode not in ('position'):
            self.core.get_logger().error(
                "Please set the gripper's 'operating mode' to 'position."
            )
            sys.exit(1)

        time.sleep(0.5)
        self.core.get_logger().info(
            (
                '\n'
                f'\tGripper Name: {self.gripper_name}\n'
            )
        )
        self.core.get_logger().info('Initialized InterbotixGripperXSInterface!')

    def set_gripper_position(self, effort: float, delay=0.0) -> None:
        """
        Publish effort commands to the gripper.

        :param effort: effort command to send to the gripper motor
        :param delay: number of seconds to wait before returning control to the user
        """
        self.gripper_command.cmd = effort
        # update gripper position
        with self.core.js_mutex:
            gripper_pos = self.core.joint_states.position[self.left_finger_index]
        # check if the gripper is within its limits
        if self.gripper_command.cmd >= -2.51 and (gripper_pos < self.left_finger_upper_limit or gripper_pos > self.left_finger_lower_limit):
            self.core.pub_single.publish(self.gripper_command)
            time.sleep(delay)

    # leaving this here in case we want to use time based control for the gripper motor eventually
    def set_trajectory_time(
        self,
        moving_time: float = None,
        accel_time: float = None
    ) -> None:
        """
        Command the 'Profile_Velocity' and 'Profile_Acceleration' motor registers.

        :param moving_time: (optional) duration in seconds that the robot should move if using time based profile
        :param accel_time: (optional) duration in seconds that that robot should spend
            accelerating/decelerating (must be less than or equal to half the moving_time) if using time based profile
        """
        self.core.get_logger().debug(
            f'Updating timing params: {moving_time=}, {accel_time=}'
        )
        if moving_time is not None and moving_time != self.moving_time:
            self.moving_time = moving_time
            future_moving_time = self.core.srv_set_reg.call_async(
                RegisterValues.Request(
                    cmd_type='single',
                    name=self.gripper_name,
                    reg='Profile_Velocity',
                    value=int(moving_time * 1000),
                )
            )
            self.core.executor.spin_once_until_future_complete(
                future=future_moving_time,
                timeout_sec=0.1
            )

        if accel_time is not None and accel_time != self.accel_time:
            self.accel_time = accel_time
            future_accel_time = self.core.srv_set_reg.call_async(
                RegisterValues.Request(
                    cmd_type='single',
                    name=self.gripper_name,
                    reg='Profile_Acceleration',
                    value=int(accel_time * 1000),
                )
            )
            self.core.executor.spin_once_until_future_complete(
                future=future_accel_time,
                timeout_sec=0.1
            )
