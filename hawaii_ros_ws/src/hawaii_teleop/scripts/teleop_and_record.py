import time
import os
import h5py
import numpy as np
from tqdm import tqdm
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

from real_env import make_real_env
from teleop_utils import get_joint_states, move_grippers, DT, STUDENT_GRIPPER_JOINT_OPEN, STUDENT_GRIPPER_JOINT_CLOSE

import IPython
e = IPython.embed

# Function to continuously read data from the serial port
# def read_from_serial(serial_port, data_queue):
#     # will need to do more once the payload structure is finalized
#     while True:
#         if serial_port.in_waiting > 0:
#             data = serial_port.readline().decode().strip()
#             data_queue.put(data)  # Put the data into the shared queue

def print_dt_diagnosis(actual_dt_history):
    actual_dt_history = np.array(actual_dt_history)
    get_action_time = actual_dt_history[:, 1] - actual_dt_history[:, 0]
    step_env_time = actual_dt_history[:, 2] - actual_dt_history[:, 1]
    total_time = actual_dt_history[:, 2] - actual_dt_history[:, 0]

    dt_mean = np.mean(total_time)
    freq_mean = 1 / dt_mean
    print(f'Avg freq: {freq_mean:.2f} Get action: {np.mean(get_action_time):.3f} Step env: {np.mean(step_env_time):.3f}')
    return freq_mean

def capture_one_episode(dt, max_timesteps, camera_names, dataset_dir, dataset_name, overwrite):

    # saving dataset
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    dataset_path = os.path.join(dataset_dir, dataset_name)
    if os.path.isfile(dataset_path) and not overwrite:
        print(f'Dataset already exist at \n{dataset_path}\nHint: set overwrite to True.')
        exit()

    env = make_real_env()

    # move all 4 robots to a starting pose where it is easy to start teleoperation, then wait till both gripper closed
    #opening_ceremony(master_bot_left, master_bot_right, env.puppet_bot_left, env.puppet_bot_right)  

    # Data collection
    ts = env.reset(fake=True)
    timesteps = [ts]
    actions = []
    actual_dt_history = []
    for t in tqdm(range(max_timesteps)):
        t0 = time.time() #
        action = get_joint_states()
        t1 = time.time() #
        ts = env.step(action)
        t2 = time.time() #
        timesteps.append(ts)
        actions.append(action)
        actual_dt_history.append([t0, t1, t2])

    # Open puppet grippers
    move_grippers([env.student_left, env.student_right], [STUDENT_GRIPPER_JOINT_OPEN] * 2, move_time=0.5)

    freq_mean = print_dt_diagnosis(actual_dt_history)
    if freq_mean < 42:
        return False
    
    """
    For each timestep:
    observations
    - images
        - cam_high          (480, 640, 3) 'uint8'
        - cam_low           (480, 640, 3) 'uint8'
        - cam_left_wrist    (480, 640, 3) 'uint8'
        - cam_right_wrist   (480, 640, 3) 'uint8'
    - qpos                  (14,)         'float64'
    - qvel                  (14,)         'float64'
    
    action                  (14,)         'float64'
    """

    data_dict = {
        '/observations/qpos': [],
        '/observations/qvel': [],
        '/observations/effort': [],
        '/action': [],
    }
    for cam_name in camera_names:
        data_dict[f'/observations/images/{cam_name}'] = []

    while actions:
        action = actions.pop(0)
        ts = timesteps.pop(0)
        data_dict['/observations/qpos'].append(ts.observation['qpos'])
        data_dict['/observations/qvel'].append(ts.observation['qvel'])
        data_dict['/observations/effort'].append(ts.observation['effort'])
        data_dict['/action'].append(action)
        for cam_name in camera_names:
            data_dict[f'/observations/images/{cam_name}'].append(ts.observation['images'][cam_name])

    # HDF5
    t0 = time.time()
    with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
        root.attrs['sim'] = False
        obs = root.create_group('observations')
        image = obs.create_group('images')
        for cam_name in camera_names:
            _ = image.create_dataset(cam_name, (max_timesteps, 480, 640, 3), dtype='uint8',
                                     chunks=(1, 480, 640, 3), )
        _ = obs.create_dataset('qpos', (max_timesteps, 14))
        _ = obs.create_dataset('qvel', (max_timesteps, 14))
        _ = obs.create_dataset('effort', (max_timesteps, 14))
        _ = root.create_dataset('action', (max_timesteps, 14))

        for name, array in data_dict.items():
            root[name][...] = array
    print(f'Saving: {time.time() - t0:.1f} secs')

    return True


if __name__=='__main__':
    # Shared queue for passing data between threads
    # shared_data_queue = queue.Queue()

    # # Create threads for reading and processing in Thread 1 and Thread 2
    # write_thread = threading.Thread(target=mock_serial_data, args=(shared_data_queue,))
    # process_thread_1 = threading.Thread(target=teleop, args=(shared_data_queue, True))

    # # Start all threads
    # write_thread.start()
    # process_thread_1.start()

    # write_thread.join()
    # process_thread_1.join()
    overwrite = True
    max_timesteps= 1000
    dataset_name = "testing"
    dataset_dir = "testing"
    camera_names= ['cam_high', 'cam_low', 'cam_left_wrist', 'cam_right_wrist']

    while True:
        is_healthy = capture_one_episode(DT, max_timesteps, camera_names, dataset_dir, dataset_name, overwrite)
        if is_healthy:
            break