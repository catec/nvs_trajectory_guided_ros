#!/usr/bin/env python3

import os
from shutil import copyfile
from PIL import Image
from scipy.spatial.transform import Rotation as R
import numpy as np
import math 

"""
Covert a quaternion into a full three-dimensional rotation matrix.

Parameters
----------
Input
:param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 

Output
:return: A 3x3 element matrix representing the full 3D rotation matrix. 
            This rotation matrix converts a point in the local reference 
            frame to a point in the global reference frame.
"""
def quaternion_rotation_matrix(Q):
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

'''
Create folder to save AirSim dataset
'''
def create_airsim_dataset_folder(dataset_folder):
    if not os.path.exists(dataset_folder):
        os.makedirs(dataset_folder)
        os.makedirs(dataset_folder + '/rgb')
        # os.makedirs(dataset_folder + '/depth')
        os.makedirs(dataset_folder + '/segmentation')
        file = open(dataset_folder + '/poses.txt', 'w+')
        file.write("#id x y z qw qx qy qz\n")
        file.close()
        file_intrinsics = open(dataset_folder + '/intrinsics.txt', 'w+')
        file_intrinsics.write("#[fx 0 cx; 0 fy cy; 0 0 1]\n")
        file_intrinsics.close()
    print("Saving dataset to {}".format(dataset_folder))

'''
Create file to save poses there
'''
def create_poses_file(file_path):
    if (os.path.isfile(file_path)):
        os.remove(file_path)
    file = open(file_path, 'w+')
    file.write("#Coordinates in World Coordinate System\n")
    file.write("#x y z roll_rad pitch_rad yaw_rad\n")
    file.close()

'''
Save 6DoF pose in format: 'x y z roll_rad pitch_rad yaw_rad' to specific file
'''
def save_pose_to_file(poses_file_path, position, orientation):
    with open(poses_file_path, "a") as file:
        file.write("{} {} {} {} {} {}\n".format(position[0], position[1], position[2], 
                                                orientation[0], orientation[1], orientation[2]))


'''
Read txt file with the following format:
#x y z roll_rad pitch_rad yaw_rad
'''
def read_trajectory_poses_from_file(filename):
    if not os.path.isfile(filename):
        raise Exception("File {} does not exist".format(filename))
    
    with open(filename, 'r') as file:
        lines = file.readlines()
        poses = []
        for line in lines:
            if line.startswith('#'):
                continue
            else:
                pose = line.split(' ')
                pose = [float(el) for el in pose]
                poses.append(pose)
        return poses


'''
Build K matrix from camera parameters
'''
def build_K_from_params(width, height, fov):
    focal_length = 0.5 * width / math.tan(np.radians(fov/2.0))
    K = np.array([[focal_length,     0.0,       width/2.0],
                  [    0.0,      focal_length, height/2.0],
                  [    0.0,          0.0,          1.0   ]])
    return K
