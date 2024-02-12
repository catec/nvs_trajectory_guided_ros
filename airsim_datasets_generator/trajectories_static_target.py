#!/usr/bin/env python3

import os 
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from visualization.visualization_utils import plot_3d_axis, plot_frustrum
from datasets.dataset_utils import create_poses_file, save_pose_to_file
from shot_type_trajectories.trajectory_generator import *

from argparse import ArgumentParser

def obtain_orbit_trajectory() -> list:

    p_tgt_wcs = np.array([-66.60, 69.7, 00.0])
    v_tgt_wcs = np.array([0.0, 0.0, 0.0])
    x0_wcs    = np.array([-65.0, 70.0, 10.0])
    
    curr_tgt_init_dist = np.linalg.norm(p_tgt_wcs - x0_wcs)
    
    lambda_dist = 15.0
    theta_deg   = 360.0
    w_uav       = 60.0
    if curr_tgt_init_dist > lambda_dist:
        print("The initial distance between the target and the UAV is bigger than the desired distance")
        print("The initial distance is {} and the desired distance is {}".format(curr_tgt_init_dist, lambda_dist))
        exit()

    generator = OrbitTrajectoryGenerator()
    generator.set_camera_framerate(30.0)
    generator.setup_known_parameters(uav_pos_wcs = x0_wcs, 
                                     tgt_pos_wcs = p_tgt_wcs,
                                     tgt_vel_wcs = v_tgt_wcs)
    xt_wcs, gimbal_ea_wcs = generator.generate_trajectory(desired_euclidean_dist = lambda_dist,
                                                          angle_rotation_performed = theta_deg,
                                                          desired_angular_vel = w_uav)

    return (p_tgt_wcs, xt_wcs, gimbal_ea_wcs)

def obtain_flyby_trajectory() -> list:
    p_tgt_wcs = np.array([110.0, 8.0, 20.0])
    v_tgt_wcs = np.array([0.0, 0.0, 0.0])
    x0_wcs    = np.array([150.0, -50.0, 20.0])

    K_sec       = 1.0
    proj_length = 15.0
    if (K_sec < 0.0):
        print("The K_sec value must be positive")
        exit()

    generator = FlyByTrajectoryGenerator()
    generator.set_camera_framerate(15.0)
    generator.setup_known_parameters(uav_pos_wcs = x0_wcs, 
                                     tgt_pos_wcs = p_tgt_wcs,
                                     tgt_vel_wcs = v_tgt_wcs)
    xt_wcs, gimbal_ea_wcs = generator.generate_trajectory(K = K_sec,
                                                          proj_length = proj_length)
    return (p_tgt_wcs, xt_wcs, gimbal_ea_wcs)

def obtain_flyover_trajectory() -> list:
    p_tgt_wcs = np.array([0.0, 0.0, 10.0])
    v_tgt_wcs = np.array([0.0, 0.0, 0.0])
    x0_wcs    = np.array([30.0, 5.0, 20.0])


    K_sec = 15.0
    if (K_sec < 0.0):
        print("The K_sec value must be positive")
        exit()

    generator = FlyOverTrajectoryGenerator()
    generator.set_camera_framerate(15.0)
    generator.setup_known_parameters(uav_pos_wcs = x0_wcs, 
                                     tgt_pos_wcs = p_tgt_wcs,
                                     tgt_vel_wcs = v_tgt_wcs)
    xt_wcs, gimbal_ea_wcs = generator.generate_trajectory(K = K_sec)
    return (p_tgt_wcs, xt_wcs, gimbal_ea_wcs)


if __name__ == "__main__":

    parser = ArgumentParser(description="Generate specific trajectories over static targets")
    parser.add_argument('--traj_type', type=str, choices=["orbit", "flyover", "flyby"], default="orbit")
    parser.add_argument('--output_file_path', type=str, default=os.getcwd())
    args = parser.parse_args()

    if args.traj_type == "orbit":
        (p_tgt_wcs, xt_wcs, gimbal_ea_wcs) = obtain_orbit_trajectory()
    elif args.traj_type == "flyby":
        (p_tgt_wcs, xt_wcs, gimbal_ea_wcs) = obtain_flyby_trajectory()
    elif args.traj_type == "flyover":
        (p_tgt_wcs, xt_wcs, gimbal_ea_wcs) = obtain_flyover_trajectory()
    else:
        print("The trajectory type {} is not implemented yet".format(args.traj_type))
        exit()
        
    # Create the file to save the poses
    trajectory_poses_file = args.output_file_path + '/trajectory_poses.txt'
    create_poses_file(trajectory_poses_file)

    # Create a figure and a 3D axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    plot_3d_axis(ax, p_tgt_wcs, np.array([0, 0, 0]), "target", 5.0)
    for uav_pos, gimbal_ori in zip(xt_wcs, gimbal_ea_wcs):
        save_pose_to_file(trajectory_poses_file, uav_pos, gimbal_ori)
        
        gimbal_ori[1] = gimbal_ori[1] + math.pi/2.0
        plot_3d_axis(ax, uav_pos, gimbal_ori, scale=3.0)
        
    print(f"Saved trajectory of type '{args.traj_type}' with lenght {len(xt_wcs)} in {trajectory_poses_file}")

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Generated trajectory')
    
    # Show the plot
    plt.show()