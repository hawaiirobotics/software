DATA_DIR = '../saved_training_data'
TASK_CONFIGS = {
    'default_task':{
        'dataset_dir': DATA_DIR + '/default_task',
        'num_episodes': 2,
        'episode_len': 1000,
        'camera_names': ['cam_high', 'cam_front', 'cam_left', 'cam_right']
    },
}
