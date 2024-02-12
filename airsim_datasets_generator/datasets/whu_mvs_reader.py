#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from visualization.visualization_utils import plot_3d_axis, plot_frustrum

import os

def read_camera_parameters(filename):
    with open(filename, "r") as f:
        lines = f.readlines()
        lines = [line.rstrip() for line in lines]
    extrinsic = np.fromstring(" ".join(lines[1:5]), dtype=np.float32, sep=" ").reshape(4, 4)
    intrinsic = np.fromstring(" ".join(lines[7:10]), dtype=np.float32, sep=" ").reshape(3, 3)
    depth_min, depth_max = [float(item) for item in lines[11].split()]
    return intrinsic, extrinsic, depth_min, depth_max

positions = []
eas = []

positions_folder = '/home/mmontes/Documents/AUTH/whu_mvs/scene_000/Cams'
for root, _, files in os.walk(os.path.abspath(positions_folder)):
    files.sort()
    for file in files:
        intrinsic, extrinsics, depth_min, depth_max = read_camera_parameters(os.path.join(root, file))

        pos = extrinsics[:3, 3]

        rot = extrinsics[:3, :3]
        ea = R.from_matrix(rot).as_euler('xyz', degrees=False)

        positions.append(pos)
        eas.append(ea)


# Create a figure and a 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

i=0
for pos, ea in zip(positions, eas):
    # if i%5 == 0:
    plot_3d_axis(ax, pos, np.array([0, 0, 0]), scale=5.0)
    plot_frustrum(ax, pos, ea, scale=5.0)
    i+=1

# # # Set the axis limits
ax.set_ylim([500, 1000])
ax.set_zlim([1260, 1280])

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Axis')

# Show the plot
plt.show()
