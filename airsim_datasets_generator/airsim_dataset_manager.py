#!/usr/bin/env python3

# In settings.json first activate computer vision mode:
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

import airsim

import datetime
from argparse import ArgumentParser
import numpy as np
import cv2
import os
import time

from datasets.dataset_utils import create_airsim_dataset_folder, read_trajectory_poses_from_file, build_K_from_params
from utils.keyboard_movement_manager import KeyboardController

'''
Class to record computer vision in Unreal Engine using AirSim API.
'''
class AirsimDatasetManager:
    def __init__(self, args) -> None:
        client = airsim.VehicleClient()
        client.confirmConnection()
        self.__client = client

        '''
        We are attaching the camera to the origin of the vehicle, so we only 
        need to move the vehicle to change the camera pose.
        '''
        initial_camera_pose = airsim.Pose(airsim.Vector3r(0.0, 0.0, 0.0), 
                                          airsim.to_quaternion(0.0, 0.0, 0.0)) #radians
        self.__client.simSetCameraPose("0", initial_camera_pose)

        cv2.startWindowThread()
        cv2.namedWindow("RGB Image",   cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Segmentation Image", cv2.WINDOW_AUTOSIZE)
        
        self.__curr_imgs_id = 0
        self.__save_dataset = args.save_dataset
        if (self.__save_dataset):
            if args.dataset_folder.endswith('/'):
                args.dataset_folder = args.dataset_folder[:-1]
            
            self.__dataset_folder = args.dataset_folder + '/dataset_{}'.format(datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S"))
            create_airsim_dataset_folder(self.__dataset_folder)

        self.requests = [airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
                        #  airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True, False),
                         airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)]

        if (self.__save_dataset):
            intrinsics = self.get_camera_intrinsics(0)
            with open(self.__dataset_folder + '/intrinsics.txt', "ab") as file:
                np.savetxt(file, intrinsics, fmt='%.6f', delimiter=' ')
                file.write(b"\n")

    def __del__(self) -> None:
        # currently reset() doesn't work in CV mode. Below is the workaround
        self.__client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(0.0, 0.0, -50.0), 
                                                    airsim.to_quaternion(0.0, 0.0, 0.0)), True)
        cv2.destroyAllWindows()

    '''
    Method to get camera intrinsic matrix in np array format
    '''
    def get_camera_intrinsics(self, camera_name) -> np.array:
        raw_img = self.__client.simGetImage(str(camera_name),airsim.ImageType.Scene)
        if not raw_img:
            print("Unable to obtain image to estimate intrinsics")
            return
        png = cv2.imdecode(airsim.string_to_uint8_array(raw_img), cv2.IMREAD_UNCHANGED)
        height, width, _ = png.shape
        fov = self.__client.simGetCameraInfo(str(camera_name)).fov

        K_mat = build_K_from_params(width, height, fov)
        return K_mat
    
    def __read_depth_image(self, data_array, height, width):
        img_depth = np.array(data_array, dtype=np.float32)
        img_depth = img_depth.reshape(height, width)
        return img_depth

    def __read_color_image(self, data_array, height, width):
        img1d = np.frombuffer(data_array, dtype=np.uint8)
        img_rgb = img1d.reshape(height, width, 3)
        return img_rgb

    '''
    Obtain images from API based on self.request configuration and convert it to OpenCV format.
    ''' 
    def get_vehicle_images(self) -> None:
        responses = self.__client.simGetImages(self.requests)
        id_str = str(self.__curr_imgs_id).zfill(4)
        for response in responses:
            if (response.compress):
                print("Unable to read compressed images. Change ImageRequest settings.")
                continue

            if (response.image_type == airsim.ImageType.DepthPlanar):
                img_depth = self.__read_depth_image(response.image_data_float, 
                                                    response.height, response.width)
                
                # normalized_depth = cv2.normalize(img_depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                # colored_depth = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)
                # cv2.imshow("Depth Image", colored_depth)

                if (self.__save_dataset):
                    cv2.imwrite(self.__dataset_folder + '/depth/depth_{}.png'.format(id_str), img_depth)

            elif (response.image_type == airsim.ImageType.Scene):
                img_rgb = self.__read_color_image(response.image_data_uint8, 
                                                  response.height, response.width)
                cv2.imshow("RGB Image", img_rgb)
                
                if (self.__save_dataset):
                    cv2.imwrite(self.__dataset_folder + '/rgb/rgb_{}.png'.format(id_str), img_rgb)

            elif (response.image_type == airsim.ImageType.Segmentation):
                img_seg = self.__read_color_image(response.image_data_uint8, 
                                                  response.height, response.width)
                cv2.imshow("Segmentation Image", img_seg)
                
                if (self.__save_dataset):
                    cv2.imwrite(self.__dataset_folder + '/segmentation/segmentation_{}.png'.format(id_str), img_seg)

            else:
                print("Error: Unrecognized image type: {}".format(response.image_type))
                return

        if (self.__save_dataset):
            curr_uav_pos = self.get_vehicle_pose().position
            curr_uav_ori = self.get_vehicle_pose().orientation

            with open(self.__dataset_folder + '/poses.txt', "a") as file:
                file.write("{} {} {} {} {} {} {} {}\n".format(id_str, curr_uav_pos.x_val, curr_uav_pos.y_val, -1.0 * curr_uav_pos.z_val,
                        curr_uav_ori.w_val, curr_uav_ori.x_val, curr_uav_ori.y_val, curr_uav_ori.z_val))

        self.__curr_imgs_id = self.__curr_imgs_id + 1
        cv2.waitKey(1)

    '''
    Update pose of simulated camera pose
    ''' 
    def update_vehicle_pose(self, position, roll_rad, pitch_rad, yaw_rad) -> None:
        self.__client.simSetVehiclePose(
            airsim.Pose(position, airsim.to_quaternion(pitch_rad, roll_rad, yaw_rad)), 
            True)

    '''
    Get pose of simulated camera pose
    ''' 
    def get_vehicle_pose(self) -> airsim.Pose:
        return self.__client.simGetVehiclePose()
    

def main(args):

    manager = AirsimDatasetManager(args)
    print("Created AirsimDatasetManager")

    if (args.use_poses_file):
        if (args.poses_path == ""):
            args.poses_path = os.getcwd() + '/trajectory_poses.txt'
        
        poses = read_trajectory_poses_from_file(args.poses_path)
        for pose in poses:
            curr_position = airsim.Vector3r(pose[0], pose[1], -1.0 * pose[2])
            manager.update_vehicle_pose(curr_position, pose[3], pose[4], pose[5])
            manager.get_vehicle_images()

            # Used to avoid overload API and get noisy images
            time.sleep(0.1)
    else:
        controller = KeyboardController(1.0, 5.0, os.getcwd(), False)
        while(controller.listener_alive()):
            if (controller.received_new_pose()):
                new_pose = controller.get_pose()
                curr_position = airsim.Vector3r(new_pose[0], new_pose[1], -1.0 * new_pose[2])
                manager.update_vehicle_pose(curr_position, np.deg2rad(new_pose[3]), np.deg2rad(new_pose[4]), np.deg2rad(new_pose[5]))
                manager.get_vehicle_images()

            # Used to avoid overload API and get noisy images
            time.sleep(0.1)

if __name__ == "__main__":
    # Implement argument parser to select stuff
    parser = ArgumentParser(description="Simulate specified trajectory in Airsim")
    parser.add_argument("--save_dataset", action="store_true", default=False)
    parser.add_argument('--dataset_folder', type=str, default=os.getcwd())
    parser.add_argument("--use_poses_file", action="store_true", default=False)
    parser.add_argument("--poses_path",  type=str, default="")

    args = parser.parse_args()
    main(args)
