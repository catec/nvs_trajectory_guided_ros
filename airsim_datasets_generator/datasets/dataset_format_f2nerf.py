#!/usr/bin/env python3

import os
from shutil import copyfile
from scipy.spatial.transform import Rotation as R
import numpy as np
from PIL import Image

from datasets.dataset_utils import *

'''
F2-NeRF format:
'''
def convert_dataset_to_f2nerf_format(dataset_folder):
    if not os.path.exists(dataset_folder):
        raise Exception("Dataset folder {} does not exist".format(dataset_folder))
    if (not os.path.isfile(dataset_folder + '/poses.txt')):
        raise Exception("Dataset folder {} does not contain poses.txt file".format(dataset_folder))
    if (not os.path.exists(dataset_folder + '/rgb')):
        raise Exception("Dataset folder {} does not contain rgb folder".format(dataset_folder))
    if (not os.path.exists(dataset_folder + '/depth')):
        raise Exception("Dataset folder {} does not contain depth folder".format(dataset_folder))
    
    f2nerf_folder = dataset_folder + '/f2nerf_format'
    if os.path.exists(f2nerf_folder):
        raise Exception("Dataset folder {} already contains f2nerf_format folder".format(dataset_folder))
    else:
        os.makedirs(f2nerf_folder)
    
    # Create Images folder
    images_folder = f2nerf_folder + '/images'
    os.makedirs(images_folder)

    curr_idx = 0
    for root, _, files in os.walk(os.path.abspath(dataset_folder + '/rgb')):
        files.sort()
        for file in files:
            copyfile(os.path.abspath(os.path.join(root, file)), 
                     os.path.join(images_folder, "{}.png".format(str(curr_idx).zfill(3))))
            curr_idx += 1

    # Obtain Depths limits
    depth_limits = [] #min ,max
    for root, _, files in os.walk(os.path.abspath(dataset_folder + '/depth')):
        files.sort()
        for file in files:
            input_depth_img = os.path.abspath(os.path.join(root, file))
            tmp_img = Image.open(input_depth_img)
            
            # With this double conversion we loose precision in the depth. IDK if it is a problem
            depth_img = np.array(tmp_img).astype(np.float32)
            depth_limits.append([np.min(depth_img), np.max(depth_img)])

    readed_poses = read_trajectory_poses_from_file(dataset_folder + '/poses.txt')
    # Default variables to create npy file
    poses             = np.zeros([len(readed_poses), 3, 4])
    intrinsics        = np.zeros([len(readed_poses), 3, 3])
    distortion_params = np.zeros([len(readed_poses), 4])
    bounds            = np.zeros([len(readed_poses), 2])

    curr_idx = 0

    # Format taken from: https://github.com/Totoro97/f2-nerf/issues/29
    for pose, depth_lims in zip(readed_poses, depth_limits):

        quaternion_gimbal = pose[4:8]  # qw qx qy qz
        r = R.from_quat([quaternion_gimbal[1], quaternion_gimbal[2], quaternion_gimbal[3], quaternion_gimbal[0]])

        # Transformation between frames
        rot_flu_to_rdf = R.from_euler('zyx', [-math.pi/2, 0.0, math.pi], degrees=False)
        R_end = np.dot(r.as_matrix(), rot_flu_to_rdf.as_matrix())

        rot_flu_to_rdf_2 = R.from_euler('zyx', [0.0, math.pi, 0.0], degrees=False)
        R_end_2 = np.dot(R_end, rot_flu_to_rdf_2.as_matrix())

        # Camera to world poses
        # OpenGL style. i.e., `negative z-axis` for looking-to, 'y-axis' for looking-up
        poses[curr_idx, :3, :3] = R_end_2             # Rotation
        poses[curr_idx, :3, 3]  = np.array(pose[1:4]) # Translation (camera position)

        # Camera intrinsic parameters.
        intrinsics[curr_idx, :, :] = build_K_from_params(width = 1280, 
                                                         height = 720, 
                                                         fov = 90.0)

        # Camera distortion parameters [k1, k2, p1, p2]
        # See https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        # If the images have been already undistorted, just set them as zero
        distortion_params[curr_idx, :] = np.zeros([4])

        # Near and far bounds of each camera along the z-axis
        print ('near {} and far {}'.format(depth_lims[0], depth_lims[1]))
        bounds[curr_idx, 0] = depth_lims[0] # Near
        bounds[curr_idx, 1] = depth_lims[1] # Far

        curr_idx += 1

    data = np.concatenate([
        poses.reshape(len(readed_poses) , 12),
        intrinsics.reshape(len(readed_poses), 9),
        distortion_params.reshape(len(readed_poses), 4),
        bounds.reshape(len(readed_poses), 2)
    ], -1)


    # Should be float64 data type
    data = np.ascontiguousarray(np.array(data).astype(np.float64))
    np.save(f2nerf_folder + '/cams_meta.npy', data)