import numpy as np
import h5py
import os

def read_hdf5_and_get_norm_values(directory_path):
    yuv_images = []

    for filename in os.listdir(directory_path):
        if filename.endswith('.hdf5'):
            file_path = os.path.join(directory_path, filename)
            with h5py.File(file_path, 'r') as file:
                num_timesteps = len(file['observations']['images']['cam_high'])

                for i in range(0, num_timesteps, 30):
                    for cam_name in ['cam_high', 'cam_front', 'cam_left', 'cam_right']:
                        if cam_name in file['observations']['images']:
                            image = file['observations']['images'][cam_name][i]
                            image = np.array(image, dtype=np.uint8)
                            yuv_images.append(image)

    # Convert images to NumPy arrays and stack
    stacked_images = np.stack([np.array(img) for img in yuv_images])

    # Compute mean and std dev for each channel
    mean = np.mean(stacked_images, axis=(0, 1, 2))
    std = np.std(stacked_images, axis=(0, 1, 2))

    print("Mean:", mean)
    print("Standard Deviation:", std)

    return mean, std

# Path to the directory containing HDF5 files
directory_path = '../saved_training_data/default_task'
mean, std = read_hdf5_and_get_norm_values(directory_path)
