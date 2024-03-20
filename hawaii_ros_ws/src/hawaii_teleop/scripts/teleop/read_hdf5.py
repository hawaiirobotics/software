import h5py
import numpy as np
import cv2

import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

def yuv422_to_rgb(image):
    yuv_image = image.reshape((480, 640, 2))
    rgb_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2RGB_YUYV)
    return rgb_image

def read_hdf5_and_display_data(file_path):
    with h5py.File(file_path, 'r') as file:
        num_timesteps = len(file['observations']['images']['cam_high'])
        
        for i in range(0, num_timesteps, 30):
            # Create a blank canvas to hold the images and qpos data
            canvas = np.zeros((480 * 2, 640 * 2, 3), dtype=np.uint8)
            
            # Display images
            for j, cam_name in enumerate(['cam_high', 'cam_front', 'cam_left', 'cam_right']):
                if cam_name in file['observations']['images']:
                    image = file['observations']['images'][cam_name][i]
                    image = np.array(image, dtype=np.uint8)
                    rgb_image = yuv422_to_rgb(image)
                    y = (j // 2) * 480
                    x = (j % 2) * 640
                    canvas[y:y+480, x:x+640] = rgb_image
            
            # Display qpos data
            qpos_data = file['observations']['qpos'][i]
            qpos_text = "qpos:\n" + "\n".join([f"{val:.4f}" for val in qpos_data])
            cv2.putText(canvas, qpos_text, (10, 970), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Display the canvas
            cv2.imshow(f"Timestep {i}", canvas)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

file_path = 'testing/testing.hdf5'
read_hdf5_and_display_data(file_path)