#!/usr/bin/env python3

from visualization_utils import plot_3d_axis, plot_frustrum

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Create a figure and a 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

drone_positions = np.array([[0.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0],
                            [0.0, 0.0, 2.0],
                            [0.0, 0.0, 3.0],
                            [0.0, 0.0, 4.0],
                            [0.0, 0.0, 5.0]
                            ])
# Euler angles (in radians) (r,p,y)
drone_orientations = np.array([[0.0, 0.0, np.radians(0.0)],
                               [0.0, 0.0, np.radians(30.0)],
                               [0.0, 0.0, np.radians(60.0)],
                               [0.0, 0.0, np.radians(90.0)],
                               [0.0, 0.0, np.radians(120.0)],
                               [0.0, 0.0, np.radians(180.0)]
                            ])
# Euler angles (in radians) (r,p,y)
gimbal_orientations = np.array([[0.0, np.radians(0.0), 0.0],
                                [0.0, np.radians(30.0), 0.0],
                                [0.0, np.radians(60.0), 0.0],
                                [0.0, np.radians(90.0), 0.0],
                                [0.0, np.radians(120.0), 0.0],
                                [0.0, np.radians(180.0), 0.0]
                            ])

# Example usage
index = 0
for uav_pos, uav_ori, gimbal_ori in zip (drone_positions, drone_orientations, gimbal_orientations):
    plot_3d_axis(ax, uav_pos, uav_ori,  label=str(index))
    plot_frustrum(ax, uav_pos, gimbal_ori, 0.3)
    index = index + 1

# Set the axis limits
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 6])

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Axis')

# Show the plot
plt.show()