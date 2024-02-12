#!/usr/bin/env python3

import math
import numpy as np

class TrajectoryGenerator:
    def __init__(self) -> None:
        pass

    '''
    Its assumed known the following parameters:
        - uav_pos_wcs: UAV position in world coordinate system
        - tgt_pos_wcs: Target position in world coordinate system
        - tgt_vel_wcs: Target velocity in world coordinate system 
    '''
    def setup_known_parameters(self, uav_pos_wcs, tgt_pos_wcs, tgt_vel_wcs):
        self._x0_wcs = uav_pos_wcs
        self._p0_wcs = tgt_pos_wcs
        self._u0_wcs = tgt_vel_wcs

    def set_camera_framerate(self, fps):
        self._cam_fps = fps

    def generate_trajectory(self, **kwargs) -> list:
        raise NotImplementedError()
    
    def _apply_tf_to_vector3(self, T, v):
        return np.matmul(T, np.append(v, 1.0))[:3]
    
    def _get_coord_systems_tfs(self, target_pos, target_vel):
        # vector n: n is orthogonal vector to Plane P (Plane P is ground plane)
        n = np.array([0.0, 0.0, 1.0])
        
        i_vec = np.array([0.0, 0.0, 0.0])
        if np.linalg.norm(target_vel) == 0:
            i_vec = np.array([1.0, 0.0, 0.0])
        else:
            # finding norm of the vector n 
            n_norm = np.sqrt(sum(n**2))
            # for projecting a vector onto the orthogonal vector n
            # find dot product using np.dot()
            proj_of_vel_on_n = (np.dot(target_vel, n)/n_norm**2)*n
            
            # Subtract proj_of_vel_on_n from u: 
            # this is the projection of u on Plane P
            i_vec = target_vel - proj_of_vel_on_n
            i_vec /= np.linalg.norm(i_vec)

        j_vec = np.cross(n, i_vec)
        k_vec = n

        j_vec /= np.linalg.norm(j_vec)
        k_vec /= np.linalg.norm(k_vec)

        # Create the rotation matrix
        r = np.eye(3)
        r[:, 0] = i_vec
        r[:, 1] = j_vec
        r[:, 2] = k_vec

        # Create T wcs->tcs
        T_tcs_to_wcs = np.eye(4)
        T_tcs_to_wcs[:3, :3] = r
        T_tcs_to_wcs[:3, 3]  = target_pos

        # Create T wcs->tcs
        T_wcs_to_tcs = np.eye(4)
        T_wcs_to_tcs[:3, :3] = np.linalg.inv(r)
        T_wcs_to_tcs[:3, 3]  = -np.dot(np.linalg.inv(r), target_pos)

        return T_wcs_to_tcs, T_tcs_to_wcs
    
    def _look_at(self, from_point, to_point, T = None):
        # Convert points to numpy arrays
        from_point = np.array(from_point)
        to_point = np.array(to_point)

        # Calculate the direction vector
        direction = to_point - from_point

        # Normalize the direction vector
        direction /= np.linalg.norm(direction)

        # Transform the direction vector between coordinate systems
        if T is not None:
            direction = np.matmul(T[:3, :3], direction)

        # Calculate the rotation angles
        yaw = np.arctan2(direction[1], direction[0])
        pitch = np.arctan2(direction[2], np.linalg.norm(direction[:2]))

        # Calculate the roll angle (around the depth axis)
        roll = 0.0  # Assume no roll since it's not determined by two points only
        return (roll, pitch, yaw)


'''
Generate an ORBIT trajectory around a target point.

Take as input the following parameters:
    - desired_euclidean_dist: Desired euclidean distance between UAV and target
    - angle_rotation_performed: Angle rotation performed by the UAV around the target
    - desired_angular_vel: Desired angular velocity of the UAV around the target

Returns: 
    - xt_wcs: UAV trajectory in WCS
    - gimbal_ea_wcs: Orientation of the gimbal in WCS in radians(roll, pitch, yaw)
'''
class OrbitTrajectoryGenerator(TrajectoryGenerator):
    def __init__(self) -> None:
        print("OrbitTrajectoryGenerator")

    def generate_trajectory(self, **kwargs) -> list:
        desired_euclidean_dist   = kwargs['desired_euclidean_dist']
        angle_rotation_performed = kwargs['angle_rotation_performed']
        desired_angular_vel      = kwargs['desired_angular_vel']

        # Define the time range
        time_limit = int ((self._cam_fps * angle_rotation_performed)/desired_angular_vel)
        sim_time = range(0, time_limit)

        # Obtain transformation matrices between WCS and TCS
        T_wcs_to_tcs, T_tcs_to_wcs = self._get_coord_systems_tfs(self._p0_wcs, self._u0_wcs)

        x0_tcs    = self._apply_tf_to_vector3(T_wcs_to_tcs, self._x0_wcs)
        p_tgt_tcs = self._apply_tf_to_vector3(T_wcs_to_tcs, self._p0_wcs)

        # theta is the absolute maximum yaw camera rotation angle
        theta_init = math.atan2(x0_tcs[1], x0_tcs[0])

        # The altitude is contant during all trajectory
        x03_tcs = x0_tcs[2]
        xt3_tcs = [x03_tcs] * len(sim_time)

        # Distance
        lambda_dist = math.sqrt(desired_euclidean_dist**2 - x03_tcs**2)
        lambda_dist = [lambda_dist] * len(sim_time)

        xt_tcs        = []
        gimbal_ea_wcs = [] 
        for t, dist, z in zip(sim_time, lambda_dist, xt3_tcs):
            xx_tcs = dist * math.cos(np.radians(t * (desired_angular_vel / self._cam_fps) + theta_init))
            xy_tcs = dist * math.sin(np.radians(t * (desired_angular_vel / self._cam_fps) + theta_init))
            xz_tcs = z

            xt_tcs.append([xx_tcs, xy_tcs, xz_tcs])

            gimbal_r, gimbal_p, gimbal_y = self._look_at([xx_tcs, xy_tcs, xz_tcs], p_tgt_tcs, T_tcs_to_wcs)
            gimbal_ea_wcs.append([gimbal_r, gimbal_p, gimbal_y])
        
        xt_wcs = []
        for xt in xt_tcs:
            xt_wcs.append(self._apply_tf_to_vector3(T_tcs_to_wcs, xt))

        return (xt_wcs, gimbal_ea_wcs)


'''
FLYBY trajectory. 
On a flyby, the camera will constantly rotate to stay on an object as the aircraft flies by in a straight line.

Take as input the following parameters:
    - K: Time until the UAV is located above the target
    - proj_length: Length of the projection of that minimal distance vector onto the ground plane
Returns: 
    - xt_wcs: UAV trajectory in WCS
    - gimbal_ea_wcs: Orientation of the gimbal in WCS in radians(roll, pitch, yaw)
'''
class FlyByTrajectoryGenerator(TrajectoryGenerator):
    def __init__(self) -> None:
        print("FlyByTrajectoryGenerator")

    def generate_trajectory(self, **kwargs) -> list:
        K_sec = kwargs['K']
        d     = kwargs['proj_length']

        # Define the time range
        time_limit = int(2 * K_sec * self._cam_fps)
        sim_time   = np.arange(0, time_limit)

        T_wcs_to_tcs, T_tcs_to_wcs = self._get_coord_systems_tfs(self._p0_wcs, self._u0_wcs)

        u0_tcs = self._apply_tf_to_vector3(T_wcs_to_tcs, self._u0_wcs)
        x0_tcs = self._apply_tf_to_vector3(T_wcs_to_tcs, self._x0_wcs)

        # Define the initial velocity in TCS frame
        v0_tcs = np.array([((u0_tcs[0] * K_sec) - x0_tcs[0]) / K_sec, 0.0, u0_tcs[2]])

        # Compute target and uav velocity in WCS frame
        vt_wcs = []
        vt_wcs.append(self._apply_tf_to_vector3(T_tcs_to_wcs, v0_tcs))
        for i in range(1, len(sim_time)):
            vt_wcs.append(vt_wcs[i-1])
        ut_wcs = []
        ut_wcs.append(self._u0_wcs)
        for i in range(1, len(sim_time)):
            ut_wcs.append(ut_wcs[i-1])

        # FLYBY definition
        xt_tsc = np.zeros((len(sim_time), 3))

        x0_tcs[1]    = d
        xt_tsc[:, 1] = x0_tcs[1]

        xKT_tcs = np.array([0.0, x0_tcs[1], x0_tcs[2]])
        xKT_wcs = self._apply_tf_to_vector3(T_tcs_to_wcs, xKT_tcs)

        # Define the 3D position of the UAV in the WCS
        xt_wcs = []
        gimbal_ea_wcs = [] 
        for t in sim_time:
            curr_pos_wcs = self._x0_wcs + (t/(self._cam_fps * K_sec)) * (xKT_wcs - self._x0_wcs)
            gimbal_r, gimbal_p, gimbal_y = self._look_at(curr_pos_wcs, self._p0_wcs)

            xt_wcs.append(curr_pos_wcs)
            gimbal_ea_wcs.append([gimbal_r, gimbal_p, gimbal_y])

        return (xt_wcs, gimbal_ea_wcs)

'''
FLYOVER trajectory
'''
class FlyOverTrajectoryGenerator(TrajectoryGenerator):
    def __init__(self) -> None:
        print("FlyOverTrajectoryGenerator")

    def generate_trajectory(self, **kwargs) -> list:
        K_sec = kwargs['K']

        # Define the time range
        time_limit = int(2 * K_sec * self._cam_fps)
        sim_time   = np.arange(0, time_limit)

        T_wcs_to_tcs, T_tcs_to_wcs = self._get_coord_systems_tfs(self._p0_wcs, self._u0_wcs)

        u0_tcs = self._apply_tf_to_vector3(T_wcs_to_tcs, self._u0_wcs)
        x0_tcs = self._apply_tf_to_vector3(T_wcs_to_tcs, self._x0_wcs)

        # Define the initial velocity in TCS frame
        v0_tcs = np.array([((u0_tcs[0] * K_sec) - x0_tcs[0]) / K_sec, 0.0, u0_tcs[2]])

        # Compute target and uav velocity in WCS frame
        vt_wcs = []
        vt_wcs.append(self._apply_tf_to_vector3(T_tcs_to_wcs, v0_tcs))
        for i in range(1, len(sim_time)):
            vt_wcs.append(vt_wcs[i-1])
        ut_wcs = []
        ut_wcs.append(self._u0_wcs)
        for i in range(1, len(sim_time)):
            ut_wcs.append(ut_wcs[i-1])

        # FLYOVER definition
        xKT_wcs = np.array([self._p0_wcs[0] + self._u0_wcs[0] * K_sec,
                            self._p0_wcs[1] + self._u0_wcs[1] * K_sec,
                            self._p0_wcs[2] + self._u0_wcs[2] * K_sec])

        # Define the 3D position of the UAV in the WCS
        xt_wcs = []
        gimbal_ea_wcs = [] 
        for t in sim_time:
            curr_pos_wcs = self._x0_wcs + (t/(self._cam_fps * K_sec)) * (xKT_wcs - self._x0_wcs)
            gimbal_r, gimbal_p, gimbal_y = self._look_at(curr_pos_wcs, self._p0_wcs)

            xt_wcs.append(curr_pos_wcs)
            gimbal_ea_wcs.append([gimbal_r, gimbal_p, gimbal_y])

        return (xt_wcs, gimbal_ea_wcs)