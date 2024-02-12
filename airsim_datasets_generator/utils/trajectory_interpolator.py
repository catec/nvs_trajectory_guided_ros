#!/usr/bin/env python3

import os 
import sys

from argparse import ArgumentParser

uppath = lambda _path, n: os.sep.join(_path.split(os.sep)[:-n])
sys.path.append(uppath(__file__, 2))

from datasets.dataset_utils import read_trajectory_poses_from_file, save_pose_to_file, create_poses_file


def __create_vec_btw_points(curr_point, tgt_point, num_steps):
    # Calculate step size for each axis
    step_p1 = (tgt_point[0] - curr_point[0]) / num_steps
    step_p2 = (tgt_point[1] - curr_point[1]) / num_steps
    step_p3 = (tgt_point[2] - curr_point[2]) / num_steps

    vec = []
    vec.append(curr_point)

    p1 = curr_point[0]
    p2 = curr_point[1]
    p3 = curr_point[2]
    for _ in range(num_steps):
        p1 += step_p1
        p2 += step_p2
        p3 += step_p3
        vec.append([p1, p2, p3])

    vec.append(tgt_point)

    return vec

def interpolate_trajectory(input_traj_path, output_traj_path, desired_len):
    poses = read_trajectory_poses_from_file(input_traj_path)
    
    if (len(poses) > desired_len):
        print(f"Input path ({len(poses)}) is longer than desired len ({desired_len})")
        return
    
    interpolated_traj = []
    step_between_poses = int(desired_len / (len(poses) - 1))
    
    curr_pose = poses.pop(0)
    curr_pos = curr_pose[0:3]
    curr_ori = curr_pose[3:6]
    for pose in poses:
        position_vec = __create_vec_btw_points(curr_pos, pose[0:3], step_between_poses)
        curr_pos = position_vec.pop(-1)

        orientation_vec = __create_vec_btw_points(curr_ori, pose[3:6], step_between_poses)
        curr_ori = orientation_vec.pop(-1)

        movement_vec = list(zip(position_vec, orientation_vec))
        interpolated_traj = interpolated_traj + movement_vec

    # Save interpolated trajectory to a file 
    create_poses_file(output_traj_path)
    for position, orientation in interpolated_traj:
        save_pose_to_file(output_traj_path, position, orientation)

    print(f"Saved trajectory generated with {len(interpolated_traj)} poses to file '{output_traj_path}'")

if __name__ == "__main__":
    parser = ArgumentParser(description="Interpolate positions of trajectory to desired lenght")
    parser.add_argument('--traj_file', type=str, default=os.getcwd() + '/trajectory_poses.txt')
    parser.add_argument('--output_file', type=str, default=os.getcwd() + '/interpolated_trajectory_poses.txt')
    parser.add_argument('--desired_len', type=int)
    args = parser.parse_args()

    interpolate_trajectory(args.traj_file, args.output_file, args.desired_len)