import numpy as np
import time
import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointSingleCommand

import IPython
e = IPython.embed

DT = 0.033
STUDENT_GRIPPER_JOINT_OPEN = 0
STUDENT_GRIPPER_JOINT_CLOSE = 0.057
START_ARM_POSE = [0,0,0,0,0,0,0]

class ImageRecorder(Node):
    def __init__(self, init_node=True, is_debug=False):
        from collections import deque
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        self.is_debug = is_debug
        self.bridge = CvBridge()
        self.camera_names = ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
        if init_node:
            rclpy.init(args=None)
        super().__init__(node_name='image_recorder')
        for cam_name in self.camera_names:
            setattr(self, f'{cam_name}_image', None)
            setattr(self, f'{cam_name}_secs', None)
            setattr(self, f'{cam_name}_nsecs', None)
            if cam_name == 'cam_high':
                callback_func = self.image_cb_cam_high
            elif cam_name == 'cam_low':
                callback_func = self.image_cb_cam_low
            elif cam_name == 'cam_left_wrist':
                callback_func = self.image_cb_cam_left_wrist
            elif cam_name == 'cam_right_wrist':
                callback_func = self.image_cb_cam_right_wrist
            else:
                raise NotImplementedError
            self.create_subscription(Image, f"/usb_{cam_name}/image_raw", callback_func, 10)
            if self.is_debug:
                setattr(self, f'{cam_name}_timestamps', deque(maxlen=50))
            while getattr(self, f'{cam_name}_image') is None and rclpy.ok():
                rclpy.spin_once(self)
        time.sleep(0.5)

    def image_cb(self, cam_name, data):
        setattr(self, f'{cam_name}_image', self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough'))
        setattr(self, f'{cam_name}_secs', data.header.stamp.secs)
        setattr(self, f'{cam_name}_nsecs', data.header.stamp.nsecs)
        if self.is_debug:
            getattr(self, f'{cam_name}_timestamps').append(data.header.stamp.secs + data.header.stamp.secs * 1e-9)

    def image_cb_cam_high(self, data):
        cam_name = 'cam_high'
        return self.image_cb(cam_name, data)

    def image_cb_cam_low(self, data):
        cam_name = 'cam_low'
        return self.image_cb(cam_name, data)

    def image_cb_cam_left_wrist(self, data):
        cam_name = 'cam_left_wrist'
        return self.image_cb(cam_name, data)

    def image_cb_cam_right_wrist(self, data):
        cam_name = 'cam_right_wrist'
        return self.image_cb(cam_name, data)

    def get_images(self):
        image_dict = dict()
        for cam_name in self.camera_names:
            image_dict[cam_name] = getattr(self, f'{cam_name}_image')
        return image_dict

    def print_diagnostics(self):
        def dt_helper(l):
            l = np.array(l)
            diff = l[1:] - l[:-1]
            return np.mean(diff)
        for cam_name in self.camera_names:
            image_freq = 1 / dt_helper(getattr(self, f'{cam_name}_timestamps'))
            print(f'{cam_name} {image_freq=:.2f}')
        print()

class Recorder(Node):
    def __init__(self, side, init_node=True, is_debug=False):
        from collections import deque
        from sensor_msgs.msg import JointState
        from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand

        self.secs = None
        self.nsecs = None
        self.qpos = None
        self.effort = None
        self.arm_command = None
        self.gripper_command = None
        self.is_debug = is_debug

        if init_node:
            rclpy.init(args=None)
        super().__init__(node_name='image_recorder')
        self.create_subscription(JointState, f'/student_{side}/joint_states', self.student_state_cb, 10)
        self.create_subscription(JointGroupCommand, f'/student_{side}/commands/joint_group', self.student_arm_commands_cb, 10)
        self.create_subscription(JointSingleCommand, f'/student_{side}/commands/joint_single', self.student_gripper_commands_cb, 10)
        if self.is_debug:
            self.joint_timestamps = deque(maxlen=50)
            self.arm_command_timestamps = deque(maxlen=50)
            self.gripper_command_timestamps = deque(maxlen=50)
        while self.qpos is None and rclpy.ok():
            rclpy.spin_once(self)
        time.sleep(0.1)

    def student_state_cb(self, data):
        print("DATA:", data)
        self.qpos = data.position
        self.qvel = data.velocity
        self.effort = data.effort
        self.data = data
        if self.is_debug:
            self.joint_timestamps.append(time.time())

    def student_arm_commands_cb(self, data):
        self.arm_command = data.cmd
        if self.is_debug:
            self.arm_command_timestamps.append(time.time())

    def student_gripper_commands_cb(self, data):
        self.gripper_command = data.cmd
        if self.is_debug:
            self.gripper_command_timestamps.append(time.time())

    def print_diagnostics(self):
        def dt_helper(l):
            l = np.array(l)
            diff = l[1:] - l[:-1]
            return np.mean(diff)

        joint_freq = 1 / dt_helper(self.joint_timestamps)
        arm_command_freq = 1 / dt_helper(self.arm_command_timestamps)
        gripper_command_freq = 1 / dt_helper(self.gripper_command_timestamps)

        print(f'{joint_freq=:.2f}\n{arm_command_freq=:.2f}\n{gripper_command_freq=:.2f}\n')

def get_arm_joint_positions(bot):
    return bot.arm.core.joint_states.position[:6]

def get_arm_gripper_positions(bot):
    joint_position = bot.gripper.core.joint_states.position[6]
    return joint_position

def move_arms(bot_list, target_pose_list, move_time=1):
    num_steps = int(move_time / DT)
    curr_pose_list = [get_arm_joint_positions(bot) for bot in bot_list]
    traj_list = [np.linspace(curr_pose, target_pose, num_steps) for curr_pose, target_pose in zip(curr_pose_list, target_pose_list)]
    for t in range(num_steps):
        for bot_id, bot in enumerate(bot_list):
            bot.arm.set_joint_positions(traj_list[bot_id][t], blocking=False)
        time.sleep(DT)

def move_grippers(bot_list, target_pose_list, move_time):
    gripper_command = JointSingleCommand(name="gripper")
    num_steps = int(move_time / DT)
    curr_pose_list = [get_arm_gripper_positions(bot) for bot in bot_list]
    traj_list = [np.linspace(curr_pose, target_pose, num_steps) for curr_pose, target_pose in zip(curr_pose_list, target_pose_list)]
    for t in range(num_steps):
        for bot_id, bot in enumerate(bot_list):
            gripper_command.cmd = traj_list[bot_id][t]
            bot.gripper.core.pub_single.publish(gripper_command)
        time.sleep(DT)

def setup_student_bot(bot):
    bot.dxl.robot_reboot_motors("single", "gripper", True)
    bot.dxl.robot_set_operating_modes("group", "arm", "position")
    bot.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
    torque_on(bot)

def setup_master_bot(bot):
    bot.dxl.robot_set_operating_modes("group", "arm", "pwm")
    bot.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
    torque_off(bot)

def set_standard_pid_gains(bot):
    bot.dxl.robot_set_motor_registers("group", "arm", 'Position_P_Gain', 800)
    bot.dxl.robot_set_motor_registers("group", "arm", 'Position_I_Gain', 0)

def set_low_pid_gains(bot):
    bot.dxl.robot_set_motor_registers("group", "arm", 'Position_P_Gain', 100)
    bot.dxl.robot_set_motor_registers("group", "arm", 'Position_I_Gain', 0)

def torque_off(bot):
    bot.dxl.robot_torque_enable("group", "arm", False)
    bot.dxl.robot_torque_enable("single", "gripper", False)

def torque_on(bot):
    bot.dxl.robot_torque_enable("group", "arm", True)
    bot.dxl.robot_torque_enable("single", "gripper", True)

value = 0
increment = 0.01
increasing = True

def get_joint_states():
    # right_states = np.zeros(7) # 6 joint + 1 gripper, for two arms
    # left_states = np.zeros(7) # 6 joint + 1 gripper, for two arms
    # Arm actions
    # will come from serial, and will need to apply offset to convert encoder values to robot values
    if increasing:
        value += increment
    else:
        value -= increment
    if value >= 0.5:  
        increasing = False
    elif value <=0:
        increasing = True

    right_states = [value]*7
    left_states = [value]*7

    return right_states, left_states