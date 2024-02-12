#!/usr/bin/env python3

import numpy as np

# https://github.com/demul/extrinsic2pyramid
# https://github.com/lagadic/visp/blob/master/script/README.md

'''
Obtain the rotation matrix from euler angles.
'''
def euler_to_rotation_matrix(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R


'''
Plot the 3D axis of a camera given its position and orientation. Also add a label to each axis.
'''
def plot_3d_axis(ax, position, euler_angles,  label = None, scale=0.5):
    # Convert Euler angles to rotation matrix
    roll, pitch, yaw = euler_angles
    R = euler_to_rotation_matrix(roll, pitch, yaw)

    # Define arrow directions
    x_direction = R[:, 0]
    y_direction = R[:, 1]
    z_direction = R[:, 2]

    # Plot the x-axis
    ax.quiver(position[0], position[1], position[2], x_direction[0], x_direction[1], x_direction[2], 
              color='r', label='X', length=scale)
    # Plot the y-axis
    ax.quiver(position[0], position[1], position[2], y_direction[0], y_direction[1], y_direction[2], 
              color='g', label='Y', length=scale)
    # Plot the z-axis
    ax.quiver(position[0], position[1], position[2], z_direction[0], z_direction[1], z_direction[2], 
              color='b', label='Z', length=scale)
    
    if label is not None:
        ax.text(position[0], position[1], position[2], label, z_direction)

'''
Class to obtain the corners of near and far planes of a frustrum given
fov horizontal and vertical, pitch and scale.
'''
class FrustrumSensor():
    def __init__(self, fov_h, fov_v, scale):
        self.__fov_h = fov_h
        self.__fov_v = fov_v
        self.__far_plane_dist  = scale
        self.__near_plane_dist = 0.0

    def get_frustrum_corners_from_pose(self, origin, roll, pitch, yaw):
        view_vector  = np.array([1, 0, 0])
        up_vector    = np.array([0, 0, 1])
        right_vector = np.array([0, 1, 0])

        horizontal_fov = np.radians(self.__fov_h)
        vertical_fov   = np.radians(self.__fov_v)

        height_near_plane = 2 * np.tan(vertical_fov/2)   * self.__near_plane_dist
        width_near_plane  = 2 * np.tan(horizontal_fov/2) * self.__near_plane_dist

        height_far_plane = 2 * np.tan(vertical_fov/2)   * self.__far_plane_dist
        width_far_plane  = 2 * np.tan(horizontal_fov/2) * self.__far_plane_dist

        center_near_plane = origin + view_vector * self.__near_plane_dist
        center_far_plane  = origin + view_vector * self.__far_plane_dist

        near_top_left     = center_near_plane + (up_vector * (height_near_plane / 2)) - (right_vector * (width_near_plane / 2))
        near_top_rigth    = center_near_plane + (up_vector * (height_near_plane / 2)) + (right_vector * (width_near_plane / 2))
        near_bottom_left  = center_near_plane - (up_vector * (height_near_plane / 2)) - (right_vector * (width_near_plane / 2))
        near_bottom_rigth = center_near_plane - (up_vector * (height_near_plane / 2)) + (right_vector * (width_near_plane / 2))
        near_plane_corners = np.array([near_top_left, near_top_rigth, near_bottom_left, near_bottom_rigth])

        far_top_left     = center_far_plane + (up_vector * (height_far_plane / 2)) - (right_vector * (width_far_plane / 2))
        far_top_rigth    = center_far_plane + (up_vector * (height_far_plane / 2)) + (right_vector * (width_far_plane / 2))
        far_bottom_left  = center_far_plane - (up_vector * (height_far_plane / 2)) - (right_vector * (width_far_plane / 2))
        far_bottom_rigth = center_far_plane - (up_vector * (height_far_plane / 2)) + (right_vector * (width_far_plane / 2))
        far_plane_corners = np.array([far_top_left, far_top_rigth, far_bottom_left, far_bottom_rigth])

        # Transform to the origin to be able to rotate the points propperly
        near_corners_over_origin = near_plane_corners - origin
        far_corners_over_origin  = far_plane_corners - origin

        rotation_matrix = euler_to_rotation_matrix(roll, pitch, yaw)

        near_corners_over_origin = np.dot(rotation_matrix, near_corners_over_origin.T).T
        far_corners_over_origin  = np.dot(rotation_matrix, far_corners_over_origin.T).T

        # Untransform the points
        near_plane_corners_rot = near_corners_over_origin + origin
        far_plane_corners_rot  = far_corners_over_origin  + origin

        return near_plane_corners_rot, far_plane_corners_rot


'''
Plot the frustrum of a sensor given its position, euler angles and scale.
'''
def plot_frustrum(ax, position, euler_angles, scale):
    fov_h         = 60.0
    fov_v         = 60.0
    roll, pitch, yaw = euler_angles
    
    sensor = FrustrumSensor(fov_h, fov_v, scale)
    near_plane_corners, far_plane_corners = sensor.get_frustrum_corners_from_pose(position, roll, pitch, yaw)

    # ax.scatter(near_plane_corners[:,0], near_plane_corners[:,1], near_plane_corners[:,2], c='b', marker='o')
    # ax.scatter(far_plane_corners[:,0], far_plane_corners[:,1], far_plane_corners[:,2], c='b', marker='o')
    
    # Its extended and reordered to visualize the frustrum far plane rectangle and
    far_plane_extended = np.array([far_plane_corners[0], far_plane_corners[1], far_plane_corners[3], far_plane_corners[2], far_plane_corners[0]])
    ax.plot(far_plane_extended[:,0], far_plane_extended[:,1], far_plane_extended[:,2], c='k')
    ax.plot(near_plane_corners[:,0], near_plane_corners[:,1], near_plane_corners[:,2], c='k')
    
    corners = np.array([near_plane_corners[0], far_plane_corners[0], near_plane_corners[1], far_plane_corners[1], 
                        near_plane_corners[2], far_plane_corners[2], near_plane_corners[3], far_plane_corners[3]])
    ax.plot(corners[:,0], corners[:,1], corners[:,2], c='k')