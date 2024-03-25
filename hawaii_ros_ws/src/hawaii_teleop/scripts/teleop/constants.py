DATA_DIR = '../saved_training_data'
TASK_CONFIGS = {
    'default_task':{
        'dataset_dir': DATA_DIR + '/default_task',
        'num_episodes': 2,
        'episode_len': 1000,
        'camera_names': ['cam_high', 'cam_front', 'cam_left', 'cam_right']
    },
    'pass_block_task':{
        'dataset_dir': DATA_DIR + '/pass_block_task',
        'num_episodes': 50,
        'episode_len': 1000,
        'camera_names': ['cam_high', 'cam_front', 'cam_left', 'cam_right']
    },
    'stack_chips':{
        'dataset_dir': DATA_DIR + '/stack_chips',
        'num_episodes': 50,
        'episode_len': 1000,
        'camera_names': ['cam_high', 'cam_front', 'cam_left', 'cam_right']
    },
    'pcb_assembly':{
        'dataset_dir': DATA_DIR + '/pcb_assembly',
        'num_episodes': 50,
        'episode_len': 1250,
        'camera_names': ['cam_high', 'cam_front', 'cam_left', 'cam_right']
    },
}
