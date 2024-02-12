#!/usr/bin/env python3

import os
from shutil import copyfile
from PIL import Image
from scipy.spatial.transform import Rotation as R
import numpy as np
from PIL import Image

from datasets.dataset_utils import *

'''
ImMPI format:
'''
def convert_dataset_to_ImMPI_format(dataset_folder):
    if not os.path.exists(dataset_folder):
        raise Exception("Dataset folder {} does not exist".format(dataset_folder))
    if (not os.path.isfile(dataset_folder + '/poses.txt')):
        raise Exception("Dataset folder {} does not contain poses.txt file".format(dataset_folder))
    if (not os.path.exists(dataset_folder + '/rgb')):
        raise Exception("Dataset folder {} does not contain rgb folder".format(dataset_folder))
    if (not os.path.exists(dataset_folder + '/depth')):
        raise Exception("Dataset folder {} does not contain depth folder".format(dataset_folder))
    
    immpi_folder = dataset_folder + '/ImMPI_format'
    if os.path.exists(immpi_folder):
        raise Exception("Dataset folder {} already contains ImMPI_format folder".format(dataset_folder))
    else:
        os.makedirs(immpi_folder)
    
    # Create Images folder
    images_folder = immpi_folder + '/Images'
    os.makedirs(images_folder)

    curr_idx = 0
    for root, _, files in os.walk(os.path.abspath(dataset_folder + '/rgb')):
        files.sort()
        for file in files:
            copyfile(os.path.abspath(os.path.join(root, file)), 
                     os.path.join(images_folder, "{}.png".format(str(curr_idx).zfill(3))))
            curr_idx += 1

    # Create Depths folder
    images_folder = immpi_folder + '/Depths'
    os.makedirs(images_folder)
    
    curr_idx = 0
    depth_limits = [] #min ,max
    for root, _, files in os.walk(os.path.abspath(dataset_folder + '/depth')):
        files.sort()
        for file in files:
            input_depth_img = os.path.abspath(os.path.join(root, file))
            tmp_img = Image.open(input_depth_img)
            
            # With this double conversion we loose precision in the depth. IDK if it is a problem
            depth_img = np.array(tmp_img).astype(np.float32)
            depth_limits.append([np.min(depth_img), np.max(depth_img)])
            
            output_depth_img = os.path.join(images_folder, "{}.tiff".format(str(curr_idx).zfill(3)))
            tmp_img.save(output_depth_img, 'TIFF')
            curr_idx += 1

    # Create Poses folder
    poses_folder = immpi_folder + '/Cams'
    os.makedirs(poses_folder)

    cam_width  = 512
    cam_height = 512
    cam_fov = 90
    intrinsics_mat = build_K_from_params(cam_width, cam_height, cam_fov)

    curr_idx = 0
    poses = read_trajectory_poses_from_file(dataset_folder + '/poses.txt')
    # poses = read_trajectory_poses_from_file(os.getcwd() + '/orbit_poses.txt')
    for pose, depth_lims in zip(poses, depth_limits):

        position = np.array(pose[1:4]) #x y z
        quaternion_gimbal = pose[4:8] #qw qx qy qz
        
        rot = quaternion_rotation_matrix(quaternion_gimbal)
        r = R.from_quat([quaternion_gimbal[1], quaternion_gimbal[2], quaternion_gimbal[3], quaternion_gimbal[0]])
        # ea = r.as_euler('xyz', degrees=True) #yaw pitch roll

        # Create extrinsics matrix
        extrinsics_mat = np.eye(4)
        extrinsics_mat[:3, :3] = r.as_matrix()
        extrinsics_mat[:3, 3]  = position
        
        curr_file_path = poses_folder + "/{}.txt".format(str(curr_idx).zfill(3))
        with open(curr_file_path, "ab") as f:
            f.write(b"extrinsics:\n")
            np.savetxt(f, extrinsics_mat, fmt='%.6f', delimiter=' ')
            f.write(b"\n")

            f.write(b"intrinsics:\n")
            np.savetxt(f, intrinsics_mat, fmt='%.6f', delimiter=' ')
            f.write(b"\n")

            depth_limits_str = "{} {}\n".format(depth_lims[0], depth_lims[1])
            f.write(depth_limits_str.encode('utf-8'))
            
        curr_idx += 1
