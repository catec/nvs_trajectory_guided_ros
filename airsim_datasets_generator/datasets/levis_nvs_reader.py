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
    extrinsic = np.fromstring(",".join(lines[1:5]), dtype=np.float32, sep=",").reshape(4, 4)
    intrinsic = np.fromstring(",".join(lines[7:10]), dtype=np.float32, sep=",").reshape(3, 3)
    depth_min, depth_max = [float(item) for item in lines[11].split(",")]
    return intrinsic, extrinsic, depth_min, depth_max

positions = []
eas = []

positions_folder = '/home/mmontes/Projects/AUTH/auth_visit_scripts/dataset_2023-05-24_12:55:34/ImMPI_format/Cams'
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
print (len(positions))
plot_3d_axis(ax, np.array([0, 0, 0]), np.array([0, 0, 0]), "target", 5.0)
for pos, ea in zip(positions, eas):
    print('Current position: ', pos)
    ea[1] = ea[1] + np.pi/2.0
    print('yaw: {}, pitch: {}, roll: {}'.format(np.degrees(ea[2]), np.degrees(ea[1]), np.degrees(ea[0])))

    plot_3d_axis(ax, pos, ea, scale=2.0)
    # plot_frustrum(ax, pos, ea, 3.0)

# # # # Set the axis limits
# ax.set_xlim([-0.01, 0.01])
# ax.set_ylim([-0.01, 0.01])
# ax.set_zlim([91.7, 92])

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Axis')

# Show the plot
plt.show()
