#!/usr/bin/env python3

import h5py
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from visualization.visualization_utils import plot_3d_axis, plot_frustrum
from datasets.dataset_utils import create_poses_file, save_pose_to_file
from trajectories.trajectory_generator import *

f = h5py.File('/home/aiiacvmllab/Documents/datasets/multiview2novelview/kitti/data.hdf5')

print('Size of dataset: {}'.format(len(f.keys())))

first_id = list(f.keys())[0]

elem = f.get(first_id)

# print(elem.keys())
xt_wcs = []
x_t = []
y_t = []
z_t = []
gimbal_ea_wcs = []
for curr_id in range(0, 10000):
    if curr_id % 10 == 0:
        curr_elem = list(f.keys())[curr_id]
        image = f[curr_elem]['image'][()]/255.*2 - 1
        pose  = f[curr_elem]['pose'][()]
        # print('Curr position: {}'.format(pose[3:6]))
        # print('Curr rpy: {}'.format(pose[0:3]))
        xt_wcs.append(np.array(pose[3:6]))
        x_t.append(pose[3])
        y_t.append(pose[4])
        z_t.append(pose[5])
        gimbal_ea_wcs.append(np.array(pose[0:3]))

# Create a 4x4 subplot grid
fig, axs = plt.subplots(2,2)

# XY plot
axs[0, 0].scatter(x_t, y_t)
axs[0, 0].set_xlabel('X')
axs[0, 0].set_ylabel('Y')
axs[0, 0].set_title('XY Plot')

# XZ plot
axs[0, 1].scatter(x_t, z_t)
axs[0, 1].set_xlabel('X')
axs[0, 1].set_ylabel('Z')
axs[0, 1].set_title('XZ Plot')

# YZ plot
axs[1, 0].scatter(y_t, z_t)
axs[1, 0].set_xlabel('Y')
axs[1, 0].set_ylabel('Z')
axs[1, 0].set_title('YZ Plot')

plt.tight_layout()
plt.show()

# ax = fig.add_subplot(111, projection='3d')

# print(xt_wcs[:][0,1])

# index = 0
# for uav_pos, gimbal_ori in zip(xt_wcs, gimbal_ea_wcs):
    
#     gimbal_ori[1] = gimbal_ori[1] + math.pi/2.0
#     plot_3d_axis(ax, uav_pos, gimbal_ori, scale=0.1)
#     # plot_frustrum(ax, uav_pos, gimbal_ori, 1.0)
    
#     index = index + 1

# # Set labels and title
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_title('Generated trajectory')

# Show the plot
plt.show()


