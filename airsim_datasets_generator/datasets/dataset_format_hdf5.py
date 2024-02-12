#!/usr/bin/env python3

import os
from shutil import copyfile
from PIL import Image
from scipy.spatial.transform import Rotation as R
import numpy as np
import h5py

from datasets.dataset_utils import *

'''
HDF5 format:
'''
def convert_dataset_to_hdf5_format(dataset_folder):
    if not os.path.exists(dataset_folder):
        raise Exception("Dataset folder {} does not exist".format(dataset_folder))
    if (not os.path.isfile(dataset_folder + '/poses.txt')):
        raise Exception("Dataset folder {} does not contain poses.txt file".format(dataset_folder))
    if (not os.path.exists(dataset_folder + '/rgb')):
        raise Exception("Dataset folder {} does not contain rgb folder".format(dataset_folder))
    
    hdf5_folder = dataset_folder + '/hdf5_format'
    if os.path.exists(hdf5_folder):
        raise Exception("Dataset folder {} already contains hdf5_format folder".format(dataset_folder))
    else:
        os.makedirs(hdf5_folder)

    hf = h5py.File(hdf5_folder + '/data.hdf5', 'w')

    images_paths = []
    for root, _, files in os.walk(os.path.abspath(dataset_folder + '/rgb')):
        files.sort()
        for file in files:
            images_paths.append(os.path.abspath(os.path.join(root, file)))
    
    poses = read_trajectory_poses_from_file(dataset_folder + '/poses.txt')

    i = 0
    data_ids = []
    for image_path, pose in zip(images_paths, poses):
        curr_group_id = "00_" + str(i).zfill(6)
        curr_group = hf.create_group(curr_group_id)
        image = Image.open(image_path)
        image_resized = image.resize((256, 256))
        image_array = np.array(image_resized)

        if image_array.shape != (256, 256, 3):
            image_array = np.array(image_resized.convert('RGB'))
        curr_group.create_dataset('image', data=image_array)

        # Create pose array
        position = np.array(pose[1:4]) # x y z
        
        quaternion_gimbal = pose[4:8]  # qw qx qy qz
        r = R.from_quat([quaternion_gimbal[1], quaternion_gimbal[2], quaternion_gimbal[3], quaternion_gimbal[0]])
        ea = r.as_euler('xyz', degrees=False) #roll pitch yaw

        oneshot_pose = np.array([ea[2], ea[1], ea[0], position[0], position[1], position[2]]) #y, p, r, x, y, z
        print("oneshot_pose: {:.4}, {:.4}, {:.4}, {:.4}, {:.4}, {:.4}".format(
            oneshot_pose[0], oneshot_pose[1], oneshot_pose[2], oneshot_pose[3], oneshot_pose[4], oneshot_pose[5]))
        curr_group.create_dataset('pose', data = oneshot_pose)

        i += 1
        data_ids.append(curr_group_id)

    hf.close()

    # Create test and train splits
    np.random.shuffle(data_ids)

    train_data = data_ids[:int((len(data_ids)+1) * .80)] # Remaining 80% to training set
    test_data  = data_ids[int((len(data_ids) + 1)*.80):] # Splits 20% data to test set

    with open(hdf5_folder + '/id_train.txt', "w+") as file:
        for curr_id in train_data:
            file.write("{}\n".format(curr_id))

    with open(hdf5_folder + '/id_test.txt', "w+") as file:
        for curr_id in test_data:
            file.write("{}\n".format(curr_id))


'''
HDF5 format:
'''
def convert_multiples_dataset_to_hdf5_format(datasets_folders, target_folder):
    
    hdf5_folder = target_folder + '/hdf5_format'
    if os.path.exists(hdf5_folder):
        raise Exception("Dataset folder {} already contains hdf5_format folder".format(target_folder))
    else:
        os.makedirs(hdf5_folder)

    hf = h5py.File(hdf5_folder + '/data.hdf5', 'w')

    dataset_id = 0
    data_ids = []
    for dataset_folder in datasets_folders:
        images_paths = []
        for root, _, files in os.walk(os.path.abspath(dataset_folder + '/rgb')):
            files.sort()
            for file in files:
                images_paths.append(os.path.abspath(os.path.join(root, file)))
        
        poses = read_trajectory_poses_from_file(dataset_folder + '/poses.txt')

        i = 0
        for image_path, pose in zip(images_paths, poses):
            curr_group_id = str(dataset_id).zfill(2) + "_" + str(i).zfill(6)
            curr_group = hf.create_group(curr_group_id)
            image = Image.open(image_path)
            image_resized = image.resize((512, 512))
            image_array = np.array(image_resized)

            if image_array.shape != (512, 512, 3):
                image_array = np.array(image_resized.convert('RGB'))
            curr_group.create_dataset('image', data=image_array)

            # Create pose array
            position = np.array(pose[1:4]) # x y z
            
            quaternion_gimbal = pose[4:8]  # qw qx qy qz
            r = R.from_quat([quaternion_gimbal[1], quaternion_gimbal[2], quaternion_gimbal[3], quaternion_gimbal[0]])
            ea = r.as_euler('xyz', degrees=False) #yaw pitch roll

            oneshot_pose = np.array([ea[2], ea[1], ea[0], position[0], position[1], position[2]]) #r, p, y, x, y, z
            curr_group.create_dataset('pose', data = oneshot_pose)

            i += 1
            data_ids.append(curr_group_id)

        dataset_id += 1

    hf.close()

    # Create test and train splits
    np.random.shuffle(data_ids)

    train_data = data_ids[:int((len(data_ids) + 1) * .80)] # Remaining 80% to training set
    test_data  = data_ids[int((len(data_ids) + 1)*.80):] # Splits 20% data to test set

    with open(hdf5_folder + '/id_train.txt', "w+") as file:
        for curr_id in train_data:
            file.write("{}\n".format(curr_id))

    with open(hdf5_folder + '/id_test.txt', "w+") as file:
        for curr_id in test_data:
            file.write("{}\n".format(curr_id))