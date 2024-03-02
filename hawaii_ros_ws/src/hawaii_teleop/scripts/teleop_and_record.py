import time
import os
import h5py
import numpy as np
import rclpy
from tqdm import tqdm
import cv2
import serial
import threading
import queue

from real_env import make_real_env
from teleop_utils import move_grippers, move_arms, setup_student_bot, get_arm_gripper_positions
from teleop_utils import DT, STUDENT_GRIPPER_JOINT_OPEN, STUDENT_GRIPPER_JOINT_CLOSE

# Create a shared queue
# the queue can grow infinitely if max_size isn't set. May need to do that eventually
data_queue = queue.Queue()
setup_done = False


value = 0
increment = 0.01
increasing = True

# Function to continuously read data from the serial port
def read_from_serial(serial_port, data_queue):
    # will need to do more once the payload structure is finalized
    print("starting thread")
    try:
        serial_port.flush()
        serial_port.readline()
    except serial.SerialException as e:
        print("Error writing to serial:", e)
        serial_port.close()
        exit(1)
    first_frame = True
    try:
        while True:
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
    # while True:
        
    #         # Read a line from the serial port
    #         line = serial_port.readline().decode('utf-8').strip()

    #         # Process the received message
    #         if line:
    #             print("Received:", line)
    #             data_queue.put(line)
    #     except serial.SerialException as e:
    #         print("Error reading serial:", e)
    #         exit(1)

def get_joint_states():
    # global increasing
    # global value
    # global increment
    # # right_states = np.zeros(7) # 6 joint + 1 gripper, for two arms
    # # left_states = np.zeros(7) # 6 joint + 1 gripper, for two arms
    # # Arm actions
    # # will come from serial, and will need to apply offset to convert encoder values to robot values
    # if increasing:
    #     value += increment
    # else:
    #     value -= increment
    # if value >= 3.1:  
    #     increasing = False
    # elif value <=0:
    #     increasing = True

    # right_states = [0,0,0,0,0,0,value]
    # left_states = [value]*7

    return data_queue.get()

def setup_student_arms(student_right):
    # reboot gripper motors, and set operating modes for all motors
    # setup_student_bot(student_left)
    setup_student_bot(student_right)

    # move student arms to teacher arm position
    start_arm_qpos = get_joint_states()
    print(start_arm_qpos)
    # move_arms([student_left, student_right], [start_arm_qpos[:6],start_arm_qpos[7:-1]] , move_time=1.5)
    move_arms([student_right], [start_arm_qpos[:6]] , move_time=10)
    print("done moving to zero")
    # move grippers to starting position
    # move_grippers([student_left, student_right], [STUDENT_GRIPPER_JOINT_OPEN] * 2, move_time=0.5)
    move_grippers([student_right], [STUDENT_GRIPPER_JOINT_OPEN] , move_time=1.5)
    print("done moving gripper to home")


    # press gripper to start data collection
    # print(f'Close the teacher gripper to start')
    # close_thresh = STUDENT_GRIPPER_JOINT_CLOSE - 0.001
    # closed = False
    # while not closed:
    #     joint_pos = get_joint_states()
    #     left_pos = list(joint_pos[:7])
    #     right_pos = list(joint_pos[7:])
    #     gripper_pos_left = left_pos[-1]
    #     gripper_pos_right = right_pos[-1]
    #     if (gripper_pos_left > close_thresh) and (gripper_pos_right > close_thresh):
    #         closed = True
    #     time.sleep(DT/10)
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

def capture_one_episode(max_timesteps, camera_names, dataset_dir, dataset_name, overwrite, using_sim, ser):

    # saving dataset
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    dataset_path = os.path.join(dataset_dir, dataset_name)
    if os.path.isfile(dataset_path) and not overwrite:
        print(f'Dataset already exist at \n{dataset_path}\nHint: set overwrite to True.')
        exit()

    env = make_real_env()

    # setup student arms and move them to where teacher arms are
    setup_student_arms(env.student_right)
    global setup_done
    setup_done = True

    print("done setup")

    # Data collection
    ts = env.reset(fake=True)
    timesteps = [ts]
    actions = []
    actual_dt_history = []
    # for t in tqdm(range(max_timesteps)):
    for t in range(max_timesteps):
        if t %100 == 0:
            print(data_queue.qsize())
        t0 = time.time() #
        action = get_joint_states()
        t1 = time.time() #
        ts = env.step(action)
        t2 = time.time() #
        timesteps.append(ts)
        actions.append(action)
        actual_dt_history.append([t0, t1, t2])

    # Open student grippers
    # move_grippers([env.student_left, env.student_right], [STUDENT_GRIPPER_JOINT_OPEN] * 2, move_time=0.5)
    move_grippers([env.student_right], [STUDENT_GRIPPER_JOINT_OPEN], move_time=1.5)
    print("DONE")
        
    freq_mean = print_dt_diagnosis(actual_dt_history)
    if freq_mean < 42 and not using_sim:
        print("Sampling frequency is too low. Should be >= 42")
        return False
    
    """
    For each timestep:
    observations
    - images
        - cam_high          (480, 640, 2) 'uint8'
        - cam_low           (480, 640, 2) 'uint8'
        - cam_left_wrist    (480, 640, 2) 'uint8'
        - cam_right_wrist   (480, 640, 2) 'uint8'
    - qpos                  (14,)         'float64'
    - qvel                  (14,)         'float64'
    - effort                (14,)         'float64'
    - action                (14,)         'float64'
    """
    print("before data dict")
    data_dict = {
        '/observations/qpos': [],
        '/observations/qvel': [],
        '/observations/effort': [],
        '/action': [],
    }
    print("before for cam_name")
    for cam_name in camera_names:
        data_dict[f'/observations/images/{cam_name}'] = []

    print("before while actions")
    while actions:
        action = actions.pop(0)
        ts = timesteps.pop(0)
        data_dict['/observations/qpos'].append(ts.observation['qpos'])
        data_dict['/observations/qvel'].append(ts.observation['qvel'])
        data_dict['/observations/effort'].append(ts.observation['effort'])
        data_dict['/action'].append(action)
        for cam_name in camera_names:
            data_dict[f'/observations/images/{cam_name}'].append(ts.observation['images'][cam_name])
   
    # yuv_image_data = data_dict[f'/observations/images/cam_high'][0]
    # Convert YUV422 to BGR using OpenCV
    # using to verify camera is capturing images
    # yuv_image = np.array(yuv_image_data)
    # bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_YUYV)
    # cv2.imshow('Image', bgr_image)
    # cv2.waitKey(0) # Keep the window open until any key is pressed
    # cv2.destroyAllWindows()

    # HDF5
    print("before making hdf5 file")
    t0 = time.time()
    with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
        root.attrs['sim'] = False
        obs = root.create_group('observations')
        image = obs.create_group('images')
        for cam_name in camera_names:
            _ = image.create_dataset(cam_name, (max_timesteps, 480, 640, 2), dtype='uint8', #have 2 channels because of YUV422 encoding
                                     chunks=(1, 480, 640, 2), )
        _ = obs.create_dataset('qpos', (max_timesteps, 14))
        if using_sim :
            _ = obs.create_dataset('qvel', (max_timesteps, 0))
            _ = obs.create_dataset('effort', (max_timesteps, 0))
        else :
            _ = obs.create_dataset('qvel', (max_timesteps, 14))
            _ = obs.create_dataset('effort', (max_timesteps, 14))
        _ = root.create_dataset('action', (max_timesteps, 14))

        for name, array in data_dict.items():
            root[name][...] = array
    print(f'Saving: {time.time() - t0:.1f} secs')

    return True


if __name__=='__main__':

    os.nice(1)

    overwrite = True
    max_timesteps= 1000
    dataset_name = "testing"
    dataset_dir = "testing"
    camera_names= ['cam_high']#, 'cam_low', 'cam_left_wrist', 'cam_right_wrist']
    using_sim = False

    # Serial port configuration
    ser_port = '/dev/ttyACM0'  # Change this to your serial port (e.g., /dev/ttyUSB0, /dev/ttyS0)
    baud_rate = 250000

    # Open the serial port
    ser = serial.Serial(ser_port, baud_rate, timeout=1)
    time.sleep(2)

    # Create one thread
    read_thread = threading.Thread(target=read_from_serial, args=(ser,data_queue))

    # Start the thread
    read_thread.start()
    
    while True:
        is_healthy = capture_one_episode(max_timesteps, camera_names, dataset_dir, dataset_name, overwrite, using_sim, ser)
        if is_healthy:
            break
    rclpy.shutdown()
    read_thread.join()
    ser.close()
