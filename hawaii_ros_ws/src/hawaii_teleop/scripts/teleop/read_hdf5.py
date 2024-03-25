import h5py
import numpy as np
import cv2
import matplotlib.pyplot as plt
import os

import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

def read_hdf5_and_display_data(file_path):
    with h5py.File(file_path, 'r') as file:
        num_timesteps = len(file['observations']['images']['cam_high'])

        # Collect qpos data for all timesteps
        all_qpos_data = np.array(file['observations']['qpos'])
        print(num_timesteps)
        
        for i in range(0, num_timesteps, 1249):
            canvas = np.zeros((480 * 2, 640 * 2, 3), dtype=np.uint8)
            
            for j, cam_name in enumerate(['cam_high', 'cam_front', 'cam_left', 'cam_right']):
                if cam_name in file['observations']['images']:
                    image = file['observations']['images'][cam_name][i]
                    image = np.array(image, dtype=np.uint8)
                    # If needed: rgb_image = yuv422_to_rgb(image)
                    rgb_image = image  # Use this if no conversion is needed
                    y = (j // 2) * 480
                    x = (j % 2) * 640
                    canvas[y:y+480, x:x+640] = rgb_image

            qpos_data = file['observations']['qpos'][i]
            qpos_text = "qpos:\n" + "\n".join([f"{val:.4f}" for val in qpos_data])
            cv2.putText(canvas, qpos_text, (10, 970), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Convert canvas from BGR (OpenCV default) to RGB for Matplotlib
            # canvas_rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
            plt.imshow(canvas)
            plt.show()

        # Plot qpos data
        plt.figure(figsize=(12, 6))
        for qpos_idx in range(all_qpos_data.shape[1]):
            plt.plot(all_qpos_data[:, qpos_idx], label=f'qpos {qpos_idx}')
        plt.xlabel('Timestep')
        plt.ylabel('qpos value')
        plt.title('qpos over Time')
        plt.legend()
        plt.show()

# directory = "../saved_training_data/pcb_assembly"
# for filename in os.listdir(directory):
#     if filename.endswith(".hdf5"):
#         input()
#         print(filename)
#         file_path = os.path.join(directory, filename)
#         read_hdf5_and_display_data(file_path)

file_path = '../saved_training_data/pcb_assembly/episode_54.hdf5'
read_hdf5_and_display_data(file_path)