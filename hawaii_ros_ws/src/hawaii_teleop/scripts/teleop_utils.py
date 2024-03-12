import numpy as np
import time
import rclpy
from rclpy.node import Node
import cProfile, pstats, io
from interbotix_xs_msgs.msg import JointSingleCommand

DT = 0.02
STUDENT_GRIPPER_JOINT_OPEN = 3.141592
STUDENT_GRIPPER_JOINT_CLOSE = -2.4
START_ARM_POSE = [0,0,0,0,0,0,0]

class ImageRecorder(Node):
    def __init__(self, init_node=True, is_debug=False):
        from collections import deque
        from sensor_msgs.msg import Image
        # self.image_queue = image_queue
        # self.request_event = request_event
        
        self.is_debug = is_debug
        # self.bridge = CvBridge()
        self.camera_names = ['cam_high', 'cam_front', 'cam_left', 'cam_right']
        if init_node:
            rclpy.init(args=None)
        super().__init__(node_name='image_recorder')
        self.profiler = cProfile.Profile()
        for cam_name in self.camera_names:
            setattr(self, f'{cam_name}_image', None)
            setattr(self, f'{cam_name}_secs', None)
            setattr(self, f'{cam_name}_nsecs', None)
            if cam_name == 'cam_high':
                callback_func = self.image_cb_cam_high
            elif cam_name == 'cam_front':
                callback_func = self.image_cb_cam_front
            elif cam_name == 'cam_left':
                callback_func = self.image_cb_cam_left
            elif cam_name == 'cam_right':
                callback_func = self.image_cb_cam_right
            else:
                raise NotImplementedError
            self.create_subscription(Image, f"/usb_{cam_name}/image_raw", callback_func, 10)
            if self.is_debug:
                setattr(self, f'{cam_name}_timestamps', deque(maxlen=50))
        while any(getattr(self, f'{cam_name}_image') is None for cam_name in self.camera_names) and rclpy.ok(): # check that all cameras are ready and publishing to their appropriate topics
            rclpy.spin_once(self)

    # def image_cb(self, cam_name, data):
    #     self.profiler.enable()
    #     setattr(self, f'{cam_name}_image', data)
    #     setattr(self, f'{cam_name}_secs', data.header.stamp.sec)
    #     setattr(self, f'{cam_name}_nsecs', data.header.stamp.nanosec)
    #     if self.is_debug:
    #         getattr(self, f'{cam_name}_timestamps').append(data.header.stamp.sec + data.header.stamp.nanosec * 1e-9)
    #     self.profiler.disable()

    def image_cb_cam_high(self, data):
        self.profiler.enable()
        self.cam_high_image=data
        self.cam_high_secs =data.header.stamp.sec
        self.cam_high_nsecs=data.header.stamp.nanosec
        self.profiler.disable()
        return

    def image_cb_cam_front(self, data):
        self.cam_front_image=data
        self.cam_front_secs =data.header.stamp.sec
        self.cam_front_nsecs=data.header.stamp.nanosec
        return

    def image_cb_cam_left(self, data):
        self.cam_left_image=data
        self.cam_left_secs =data.header.stamp.sec
        self.cam_left_nsecs=data.header.stamp.nanosec
        return

    def image_cb_cam_right(self, data):
        self.cam_right_image=data
        self.cam_right_secs =data.header.stamp.sec
        self.cam_right_nsecs=data.header.stamp.nanosec
        return

    def print_profiling_stats(self):
        s = io.StringIO()
        sortby = 'cumulative'
        ps = pstats.Stats(self.profiler, stream=s).sort_stats(sortby)
        ps.print_stats()
        print(s.getvalue())

    def get_images(self):
        image_dict = dict()
        for cam_name in self.camera_names:
            image_dict[cam_name] = getattr(self, f'{cam_name}_image')
        return image_dict

    # def get_images(self):
    #     if self.request_event.is_set():
    #         image_dict = dict()
    #         for cam_name in self.camera_names:
    #             image_dict[cam_name] = getattr(self, f'{cam_name}_image')
    #         self.image_queue.put(image_dict)
    #         self.request_event.clear()  # Reset the event


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
        super().__init__(node_name=f'recorder{side}')
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
            bot.arm.set_joint_positions_for_homing(traj_list[bot_id][t], blocking=False)
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
    bot.core.robot_reboot_motors("single", "gripper", True)
    torque_on(bot)

def torque_off(bot):
    bot.core.robot_torque_enable("group", "arm", False)
    bot.core.robot_torque_enable("single", "gripper", False)

def torque_on(bot):
    bot.core.robot_torque_enable("group", "arm", True)
    bot.core.robot_torque_enable("single", "gripper", True)
