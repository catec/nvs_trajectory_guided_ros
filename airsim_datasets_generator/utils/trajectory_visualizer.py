#!/usr/bin/env python3

import os 
import sys
import numpy as np
import math

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from argparse import ArgumentParser

uppath = lambda _path, n: os.sep.join(_path.split(os.sep)[:-n])
sys.path.append(uppath(__file__, 2))

from visualization.visualization_utils import plot_3d_axis, plot_frustrum
from datasets.dataset_utils import read_trajectory_poses_from_file


def visualize_3d_trajectory_from_path(traj_path):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plot_3d_axis(ax, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], label= "origin", scale=3.0)

    poses = read_trajectory_poses_from_file(traj_path)
    for pose in poses:
        pos = pose[0:3]
        ori = pose[3:6]
        
        # NOTE. Not totally sure if its necessary this offset
        # ori[1] = ori[1] + math.pi/2.0
        plot_3d_axis(ax, pos, ori, scale=1.0)
        
    
    ax.set_xlim3d(min([row[0] for row in poses]) - 5, max([row[0] for row in poses]) + 5)
    ax.set_ylim3d(min([row[1] for row in poses]) - 5, max([row[1] for row in poses]) + 5)
    ax.set_zlim3d(0.0, max([row[2] for row in poses]) + 5)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Generated trajectory')

    plt.show()


if __name__ == "__main__":

    parser = ArgumentParser(description="Visualize 3D trajectory")
    parser.add_argument('--traj_file', type=str, default=os.getcwd() + '/trajectory_poses.txt')
    args = parser.parse_args()

    visualize_3d_trajectory_from_path(args.traj_file)
