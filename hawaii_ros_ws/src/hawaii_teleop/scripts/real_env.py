import time
import numpy as np
import collections
import matplotlib.pyplot as plt
import dm_env
import rclpy
from rclpy.executors import MultiThreadedExecutor
import multiprocessing

from teleop_utils import Recorder, ImageRecorder
from threading import Thread
from teleop_utils import move_arms, START_ARM_POSE
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand

class RealEnv:
    """
    Environment for real robot bi-manual manipulation
    Action space:      [left_arm_qpos (6),             # absolute joint position
                        left_gripper_positions (1),    # normalized gripper position (0: close, 1: open)
                        right_arm_qpos (6),            # absolute joint position
                        right_gripper_positions (1),]  # normalized gripper position (0: close, 1: open)

    Observation space: {"qpos": Concat[ left_arm_qpos (6),          # absolute joint position
                                        left_gripper_position (1),  # normalized gripper position (0: close, 1: open)
                                        right_arm_qpos (6),         # absolute joint position
                                        right_gripper_qpos (1)]     # normalized gripper position (0: close, 1: open)
                        "qvel": Concat[ left_arm_qvel (6),         # absolute joint velocity (rad)
                                        left_gripper_velocity (1),  # normalized gripper velocity (pos: opening, neg: closing)
                                        right_arm_qvel (6),         # absolute joint velocity (rad)
                                        right_gripper_qvel (1)]     # normalized gripper velocity (pos: opening, neg: closing)
                        "images": {"cam_high": (480x640x3),        # h, w, c, dtype='uint8'
                                   "cam_front": (480x640x3),         # h, w, c, dtype='uint8'
                                   "cam_left_wrist": (480x640x3),  # h, w, c, dtype='uint8'
                                   "cam_right_wrist": (480x640x3)} # h, w, c, dtype='uint8'
    """
    
    def __init__(self):
        self.student_right = InterbotixManipulatorXS(robot_model="Student_Arm", group_name="arm", gripper_name="gripper", robot_name=f'student_right', init_node=True)
        self.student_left = InterbotixManipulatorXS(robot_model="Student_Arm", group_name="arm", gripper_name="gripper", robot_name=f'student_left', init_node=False)

        self.recorder_left = Recorder('left', init_node=False)
        self.recorder_right = Recorder('right', init_node=False)
        self.image_recorder = ImageRecorder(init_node=False)
        self.gripper_command = JointSingleCommand(name="gripper")

        self.start()
        # self.image_queue = multiprocessing.Queue()
        # self.request_event = multiprocessing.Event()
        # image_recorder_process = multiprocessing.Process(target=self.run_image_recorder, args=(self.image_queue, self.request_event))
        # image_recorder_process.start()
        # print("done")

    def start(self) -> None:
        """Start a background thread that builds and spins an executor."""
        self._execution_thread = Thread(target=self.run)
        self._execution_thread.start()

    def run_image_recorder(self, image_queue, request_event):
        image_recorder = ImageRecorder(image_queue, request_event, init_node=False)
        rclpy.spin(image_recorder)
        image_recorder.destroy_node()

    def run(self) -> None:
        """Thread target."""
        self.ex = MultiThreadedExecutor()
        self.ex.add_node(self.recorder_left)
        self.ex.add_node(self.recorder_right)
        self.ex.add_node(self.image_recorder) #10 fps = 60hz, 30 fps = 30 hz
        self.ex.spin()

    def shutdown(self) -> None:
        """Destroy the node and shut down all threads and processes."""
        self.recorder_left.destroy_node()
        self.recorder_right.destroy_node()
        # self.image_recorder.destroy_node()
        rclpy.shutdown()
        self._execution_thread.join()
        time.sleep(0.5)

    def get_qpos(self):
        left_qpos_raw = self.recorder_left.qpos
        right_qpos_raw = self.recorder_right.qpos
        left_arm_qpos = left_qpos_raw[:7]
        right_arm_qpos = right_qpos_raw[:7]
        return np.concatenate([left_arm_qpos, right_arm_qpos])

    def get_qvel(self):
        left_qvel_raw = self.recorder_left.qvel
        right_qvel_raw = self.recorder_right.qvel
        left_arm_qvel = left_qvel_raw[:7]
        right_arm_qvel = right_qvel_raw[:7]
        return np.concatenate([left_arm_qvel, right_arm_qvel])

    def get_effort(self):
        left_effort_raw = self.recorder_left.effort
        right_effort_raw = self.recorder_right.effort
        left_robot_effort = left_effort_raw[:7]
        right_robot_effort = right_effort_raw[:7]
        return np.concatenate([left_robot_effort, right_robot_effort])

    def get_images(self):
        return self.image_recorder.get_images()
        # print("trying to get images")
        # self.request_event.set()
        # try:
        #     images = self.image_queue.get(timeout=10)  # Adjust timeout as needed
        #     # Process the images
        # except multiprocessing.queue.Empty:
        #     print("No images received")
        # return images

    def set_gripper_pose(self, left_gripper_desired_pos_normalized, right_gripper_desired_pos_normalized):
        left_gripper_desired_joint = left_gripper_desired_pos_normalized
        self.gripper_command.cmd = left_gripper_desired_joint
        self.student_left.gripper.core.pub_single.publish(self.gripper_command)

        right_gripper_desired_joint = right_gripper_desired_pos_normalized
        self.gripper_command.cmd = right_gripper_desired_joint
        self.student_right.gripper.core.pub_single.publish(self.gripper_command)

    def _reset_joints(self):
        reset_position = START_ARM_POSE[:6]
        move_arms([self.student_left, self.student_right], [reset_position, reset_position], move_time=10)


    def _reset_gripper(self):
        """Set to position mode and do position resets: first open then close. Then change back to PWM mode"""
        # move_grippers([self.student_left, self.student_right], STUDENT_GRIPPER_JOINT_OPEN, move_time=0.5)
        # move_grippers([self.student_left, self.student_right], STUDENT_GRIPPER_JOINT_CLOSE, move_time=1)

    def get_observation(self):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos()
        obs['qvel'] = self.get_qvel()
        obs['effort'] = self.get_effort()
        obs['images'] = self.get_images()
        return obs

    def get_reward(self):
        return 0

    def reset(self, fake=False):
        if not fake:
            # Reboot student robot gripper motors
            self.student_left.core.robot_reboot_motors("single", "gripper", True)
            self.student_right.core.robot_reboot_motors("single", "gripper", True)
            self._reset_joints()
            # self._reset_gripper()
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation())

    def step(self, action):
        left_action = list(action[:7])
        right_action = list(action[7:])
        self.student_left.arm.set_joint_positions(left_action[:6], blocking=False)
        self.student_right.arm.set_joint_positions(right_action[:6], blocking=False)
        self.set_gripper_pose(left_action[-1], right_action[-1])
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation())

def make_real_env():
    env = RealEnv()
    return env
    