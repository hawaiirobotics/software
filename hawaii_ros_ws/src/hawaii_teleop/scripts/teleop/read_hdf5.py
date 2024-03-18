import h5py
import numpy as np
import matplotlib.pyplot as plt
import cv2

def yuv422_to_rgb(image):
    # Assuming image shape is (480, 640, 2), representing YUV422
    yuv_image = image.reshape((480, 640, 2))
    
    # Try different conversion if the default one doesn't work
    rgb_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2RGB_YUYV)  # Change this line to try different conversions
    return rgb_image


def read_hdf5_and_display_images(file_path):
    with h5py.File(file_path, 'r') as file:
        for i in range(0, len(file['observations']['images']['cam_high']), 30):
            for cam_name in ['cam_high']:#, 'cam_front', 'cam_left', 'cam_right']:
                image = file['observations']['images'][cam_name][i]
                image = np.array(image, dtype=np.uint8)

                # Convert YUV422 to RGB for display
                rgb_image = yuv422_to_rgb(image)

                plt.imshow(rgb_image)
                plt.title(f"{cam_name} - Image at Timestep {i}")
                plt.show()

file_path = 'testing/testing.hdf5'
read_hdf5_and_display_images(file_path)
