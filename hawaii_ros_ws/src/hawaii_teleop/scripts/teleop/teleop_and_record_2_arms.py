import time
import os
import sys
import h5py
import numpy as np
import rclpy
from tqdm import tqdm
import cv2
import serial
import threading
import queue

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))

from real_env import make_real_env
from teleop_utils import move_grippers, move_arms, reboot_gripper, torque_on, get_arm_gripper_positions, get_arm_joint_positions
from teleop_utils import STUDENT_GRIPPER_JOINT_OPEN
from cv_bridge import CvBridge
from constants import TASK_CONFIGS

# Create a shared queue
# the queue can grow infinitely if max_size isn't set. May need to do that eventually
data_queue = queue.Queue()
setup_done = False

# Function to continuously read data from the serial port
def read_from_serial(serial_port, data_queue):
    # will need to do more once the payload structure is finalized
    print("starting thread")
    global stop_threads
    try:
        serial_port.flush()
        serial_port.readline()
    except serial.SerialException as e:
        print("Error writing to serial:", e)
        serial_port.close()
        exit(1)
    first_frame = True
    try:
        while True and not stop_threads:
            if serial_port.in_waiting > 0:
                line = serial_port.readline().decode('utf-8').strip()
                elements = line.split(',')
                
                if first_frame:
                    while len(elements) != 14:
                        line = serial_port.readline().decode('utf-8').strip()
                        elements = line.split(',')

                if first_frame or setup_done:
                    first_frame = False
                    float_data = [float(element) for element in elements]
                    right_states = np.array(float_data[:7])
                    left_states = np.array(float_data[7:])
                    data_queue.put(np.concatenate((right_states, left_states)))
    except (KeyboardInterrupt, serial.SerialException):
        print("\nKeyboard interrupt received, closing serial port.")
        serial_port.close()
        exit(1)

def get_joint_commands():
    return data_queue.get()

def setup_student_arms(student_left, student_right):
    # reboot gripper motors, and set operating modes for all motors
    reboot_gripper(student_left)
    reboot_gripper(student_right)
    torque_on(student_left)
    torque_on(student_right)

    # move student arms to teacher arm position
    start_arm_qpos = get_joint_commands()

    # raise arms off of table to allow other joints to move
    start_left_teacher_pos = get_arm_joint_positions(student_left)
    start_right_teacher_pos = get_arm_joint_positions(student_right)
    start_left_teacher_pos[1] = 0.0
    start_right_teacher_pos[1] = 0.0
    move_arms([student_left, student_right], [start_left_teacher_pos,start_right_teacher_pos] , move_time=2)

    # move arms to starting position
    move_arms([student_left, student_right], [start_arm_qpos[:6],start_arm_qpos[7:-1]] , move_time=5)
    print("done moving to zero")

    # move grippers to starting position
    move_grippers([student_left, student_right], [STUDENT_GRIPPER_JOINT_OPEN, STUDENT_GRIPPER_JOINT_OPEN], move_time=1.5)
    print("done moving gripper to home")
    

    print(f'Teleop starting!')

def print_dt_diagnosis(actual_dt_history):
    actual_dt_history = np.array(actual_dt_history)
    get_action_time = actual_dt_history[:, 1] - actual_dt_history[:, 0]
    step_env_time = actual_dt_history[:, 2] - actual_dt_history[:, 1]
    total_time = actual_dt_history[:, 2] - actual_dt_history[:, 0]

    dt_mean = np.mean(total_time)
    freq_mean = 1 / dt_mean
    print(f'Avg freq: {freq_mean:.2f} Get action: {np.mean(get_action_time):.3f} Step env: {np.mean(step_env_time):.3f}')
    return freq_mean

def capture_one_episode(env, max_timesteps, camera_names, dataset_dir, dataset_name, overwrite, using_sim):
    # saving dataset
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    dataset_path = os.path.join(dataset_dir, dataset_name)
    if os.path.isfile(dataset_path) and not overwrite:
        print(f'Dataset already exist at \n{dataset_path}\nHint: set overwrite to True.')
        exit()
    

    # Clear the queue before setting up the student arms
    # while not data_queue.empty():
    #     data_queue.get()

    # setup student arms and move them to where teacher arms are
    setup_student_arms(env.student_left, env.student_right)
    global setup_done
    setup_done = True

    print("done setup")

    # Data collection
    ts = env.reset(fake=True)
    timesteps = [ts]
    actions = []
    actual_dt_history = []

    left_joint_positions = np.array(get_arm_joint_positions(env.student_left))
    left_gripper_position = get_arm_gripper_positions(env.student_left)

    right_joint_positions = np.array(get_arm_joint_positions(env.student_right))
    right_gripper_position = get_arm_gripper_positions(env.student_right)

    prev_command = np.concatenate((left_joint_positions, [left_gripper_position],
                                   right_joint_positions, [right_gripper_position]))
    threshold = 0.2

    # Clear queue before collecting data
    while not data_queue.empty():
        data_queue.get()

    env.start()
    for t in range(max_timesteps):
        if t %100 == 0:
            print(data_queue.qsize())
        t0 = time.time() #
        action = get_joint_commands()

        t1 = time.time() #
        ts = env.step(action)
        t2 = time.time() #
        if t == max_timesteps - 1:
            reboot_gripper(env.student_left)
            reboot_gripper(env.student_right)

        timesteps.append(ts)
        actions.append(action)
        actual_dt_history.append([t0, t1, t2])
    
    print("DONE")
    setup_done= False

    freq_mean = print_dt_diagnosis(actual_dt_history)
    if freq_mean < 42 and not using_sim:
        print("Sampling frequency is too low. Should be >= 42")
        return False
    
    """
    For each timestep:
    observations
    - images
        - cam_high          (480, 640, 2) 'uint8'
        - cam_front          (480, 640, 2) 'uint8'
        - cam_left          (480, 640, 2) 'uint8'
        - cam_right         (480, 640, 2) 'uint8'
    - qpos                  (14,)         'float64'
    - qvel                  (14,)         'float64'
    - effort                (14,)         'float64'
    - action                (14,)         'float64'
    """
    data_dict = {
        '/observations/qpos': [],
        '/observations/qvel': [],
        '/observations/effort': [],
        '/action': [],
    }
    for cam_name in camera_names:
        data_dict[f'/observations/images/{cam_name}'] = []

    bridge = CvBridge()
    while actions:
        action = actions.pop(0)
        ts = timesteps.pop(0)
        data_dict['/observations/qpos'].append(ts.observation['qpos'])
        data_dict['/observations/qvel'].append(ts.observation['qvel'])
        data_dict['/observations/effort'].append(ts.observation['effort'])
        data_dict['/action'].append(action)
        for cam_name in camera_names:
            data_dict[f'/observations/images/{cam_name}'].append(cv2.cvtColor(bridge.imgmsg_to_cv2(ts.observation['images'][cam_name], desired_encoding='passthrough'),cv2.COLOR_YUV2RGB_YUYV))

    t0 = time.time()
    with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
        root.attrs['sim'] = False
        obs = root.create_group('observations')
        image = obs.create_group('images')
        for cam_name in camera_names:
            _ = image.create_dataset(cam_name, (max_timesteps, 480, 640, 3), dtype='uint8', chunks=(1, 480, 640, 3), )
        _ = obs.create_dataset('qpos', (max_timesteps, 14))
        if using_sim :
            _ = obs.create_dataset('qvel', (max_timesteps, 0))
            _ = obs.create_dataset('effort', (max_timesteps, 0))
        else :
            _ = obs.create_dataset('qvel', (max_timesteps, 14))
            _ = obs.create_dataset('effort', (max_timesteps, 14))
        _ = root.create_dataset('action', (max_timesteps, 14))

        for name, array in data_dict.items():
            if array is not None:
                try:
                    root[name][...] = array
                except TypeError as e:
                    print(f"Error setting array for {name}: {e}")
                    print(f"Array type: {type(array)}, Array content: {array}")
            else:
                print(f"Array for {name} group is None")

    print(f'Saving: {time.time() - t0:.1f} secs')

    return True

def get_auto_index(dataset_dir, dataset_name_prefix = '', data_suffix = 'hdf5'):
    max_idx = 1000
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    for i in range(max_idx+1):
        if not os.path.isfile(os.path.join(dataset_dir, f'{dataset_name_prefix}episode_{i}.{data_suffix}')):
            return i
    raise Exception(f"Error getting auto index, or more than {max_idx} episodes")


if __name__=='__main__':
    os.nice(1)
    env = make_real_env()

    overwrite = True
    using_sim = False
    stop_threads = False

    task_config = TASK_CONFIGS["pcb_assembly"]
    dataset_dir = task_config['dataset_dir']
    max_timesteps = task_config['episode_len']
    camera_names = task_config['camera_names']

    episode_idx = get_auto_index(dataset_dir)
    overwrite = True

    dataset_name = f'episode_{episode_idx}'
    print(dataset_name + '\n')

    ser_port = '/dev/tty_TEACHER_ARMS' 
    baud_rate = 250000

    try:
        ser = serial.Serial(ser_port, baud_rate, timeout=1)
        print("Serial port initialized: ", ser)
    except serial.SerialException as e:
        print("Error opening serial port: ", e)
    time.sleep(2)

    read_thread = threading.Thread(target=read_from_serial, args=(ser,data_queue))
    read_thread.start()
    
    try:
        while True:
            dataset_name = f'episode_{episode_idx}'
            print(dataset_name + '\n')
            
            try:
                is_healthy = capture_one_episode(env, max_timesteps, camera_names, dataset_dir, dataset_name, overwrite, using_sim)
                print("Finished session")
                
                if not is_healthy:
                    break
                
                episode_idx += 1
            except KeyboardInterrupt:
                break
    finally:
        print("Rebooting Grippers... ")
        stop_threads = True
        read_thread.join()  # wait for thread to finish what it was doing
        rclpy.shutdown()
        ser.close()
