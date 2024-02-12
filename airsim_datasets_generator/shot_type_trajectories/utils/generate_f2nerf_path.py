#!/usr/bin/env python3

import click
import os
import numpy as np
import copy
import cv2 as cv
from os.path import join as pjoin
from glob import glob
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def inter_two_poses(pose_a, pose_b, alpha):
    ret = np.zeros([3, 4], dtype=np.float64)
    rot_a = R.from_matrix(pose_a[:3, :3])
    rot_b = R.from_matrix(pose_b[:3, :3])
    key_rots = R.from_matrix(np.stack([pose_a[:3, :3], pose_b[:3, :3]], 0))
    key_times = [0, 1]
    slerp = Slerp(key_times, key_rots)
    rot = slerp(1. - alpha)
    ret[:3, :3] = rot.as_matrix()
    ret[:3, 3] = (pose_a * alpha + pose_b * (1. - alpha))[:3, 3]
    return ret


def inter_poses(key_poses, n_out_poses, sigma=1.):
    n_key_poses = len(key_poses)
    out_poses = []
    for i in range(n_out_poses):
        w = np.linspace(0, n_key_poses - 1, n_key_poses)
        w = np.exp(-(np.abs(i / n_out_poses * n_key_poses - w) / sigma)**2)
        w = w + 1e-6
        w /= np.sum(w)
        cur_pose = key_poses[0]
        cur_w = w[0]
        for j in range(0, n_key_poses - 1):
            cur_pose = inter_two_poses(cur_pose, key_poses[j + 1], cur_w / (cur_w + w[j + 1]))
            cur_w += w[j + 1]

        out_poses.append(cur_pose)

    return np.stack(out_poses)

@click.command()
@click.option('--data_dir', type=str)
@click.option('--key_poses', type=str)
@click.option('--n_out_poses', type=int, default=240)
def hello(data_dir, n_out_poses, key_poses):
    poses = np.load(pjoin(data_dir, 'cams_meta.npy')).reshape(-1, 27)[:, :12].reshape(-1, 3, 4)

    n_poses = len(poses)

    print(poses[0])

    # key_poses = np.array([int(_) for _ in key_poses.split(',')])
    # key_poses = poses[key_poses]

    # out_poses = inter_poses(key_poses, n_out_poses)
    # print("out_poses: \n{}".format(out_poses.shape))
    # out_poses = np.ascontiguousarray(out_poses.astype(np.float64))
    # np.save(pjoin(data_dir, 'poses_render.npy'), out_poses)


    # Plot the coordinate frame
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plot_output_poses(ax, copy.deepcopy(poses), False)

    # out_inv_poses = plot_output_poses(ax, out_poses, True)
    # np.save(pjoin(data_dir, 'poses_render_inv.npy'), out_inv_poses)

    plt.show()

import math
def plot_output_poses(ax, out_poses, invert=False):
    for i in range(out_poses.shape[0]):
        R_mat = out_poses[i, :3, :3] # Rotation
        t = np.array([0, 0, 0])
        t = out_poses[i, :3, 3]  # Translation (camera position)

        # print("t: {}".format(t))
        # if invert:
        #     t[0] = -1.0
        #     t[1] = -1.0
        #     t[2] = t[2] + 5.0
        #     print("t_new: {}".format(t))


        # Define arrow directions
        x_direction = R_mat[:, 0]
        y_direction = R_mat[:, 1]
        z_direction = R_mat[:, 2]

        length = 1.0
        if invert:
            length = 0.5
        # Plot the x-axis
        ax.quiver(t[0], t[1], t[2], x_direction[0], x_direction[1], x_direction[2], 
                color='r', label='X', length=length)
        # Plot the y-axis
        ax.quiver(t[0], t[1], t[2], y_direction[0], y_direction[1], y_direction[2], 
                color='g', label='Y', length=length)
        # Plot the z-axis
        ax.quiver(t[0], t[1], t[2], z_direction[0], z_direction[1], z_direction[2], 
                color='b', label='Z', length=length)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Set the limits of the X, Y, and Z axes
    # ax.set_xlim(-5, 5)
    # ax.set_ylim(-5, 5)
    # ax.set_zlim(40, 60)

    return out_poses

if __name__ == '__main__':
    hello()
