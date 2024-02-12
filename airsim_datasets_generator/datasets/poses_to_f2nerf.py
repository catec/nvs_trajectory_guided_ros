#!/usr/bin/env python3

import numpy as np
import click
from os.path import join as pjoin
import math 

@click.command()
@click.option('--data_dir', type=str, default='.')
def main(data_dir):
    # Number of images
    n_cams = 10

    # Camera to world poses
    # OpenGL style. i.e., `negative z-axis` for looking-to, 'y-axis' for looking-up
    poses = np.zeros([n_cams, 3, 4])
    poses[:, :3, :3] = np.eye(3)                # Rotation
    poses[:, :3, 3] = np.array([0., 0., 0.])    # Translation (camera position)

    print((poses[3]))
    
    cam_width = 512
    cam_height = 512
    fov = 90.0
    focal_length = 0.5 * cam_width / math.tan(np.radians(fov/2.0))
    cx = cam_width/2.0
    cy = cam_height/2.0
    # Camera intrinsic parameters.
    intri = np.zeros([n_cams, 3, 3])
    intri[:, 0, 0] = focal_length  # fx
    intri[:, 1, 1] = focal_length  # fy
    intri[:, 0, 2] = cx            # cx
    intri[:, 1, 2] = cy            # cy
    intri[:, 2, 2] = 1.

    # Camera distortion parameters [k1, k2, p1, p2]
    # See https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
    # If the images have been already undistorted, just set them as zero
    distortion_params = np.zeros([n_cams, 4])

    # Near and far bounds of each camera along the z-axis
    bounds = np.zeros([n_cams, 2])
    bounds[:, 0] = 1.                           # Near
    bounds[:, 1] = 100.                         # Far

    data = np.concatenate([
        poses.reshape(n_cams, 12),
        intri.reshape(n_cams, 9),
        distortion_params.reshape(n_cams, 4),
        bounds.reshape(n_cams, 2)
    ], -1)

    print(data.shape)

    # Should be float64 data type
    data = np.ascontiguousarray(np.array(data).astype(np.float64))
    np.save(pjoin(data_dir, 'cams_meta.npy'), data)


if __name__ == '__main__':
    main()