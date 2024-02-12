#!/usr/bin/env python3

# In settings.json first activate computer vision mode:
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

import airsim
from airsim.utils import to_quaternion

from argparse import ArgumentParser
import time
import os
import numpy as np

'''
Class to configure environment in Unreal Engine using AirSim API.
'''
class AirsimEnvironmentManager:
    def __init__(self) -> None:
        client = airsim.VehicleClient()
        client.confirmConnection()
        self.__client = client

    '''
    Enable weather simulation
    '''
    def enable_weather(self) -> None:
        self.__client.simEnableWeather(True)

    '''
    Change environment weather to one of the following list:
        class WeatherParameter:
            Rain = 0
            Roadwetness = 1
            Snow = 2
            RoadSnow = 3
            MapleLeaf = 4
            RoadLeaf = 5
            Dust = 6
            Fog = 7
            Enabled = 8
    '''
    def change_weather(self, weather_param, percentage) -> None:
        if percentage < 0.0 or percentage > 1.0:
            print('Invalid value')
            return
        self.__client.simSetWeatherParameter(weather_param, percentage)

    '''
    Move airsim object to specified 6DoF pose.
    '''
    def move_object_to_position(self, object_name, desired_position, desired_ea_deg) -> bool:
        curr_pose = self.__client.simGetObjectPose(object_name)
        if (curr_pose.containsNan()):
            print("Unable to get object. Try updating name")
            return False

        pose = airsim.Pose()
        pose.position = airsim.Vector3r(desired_position[0], desired_position[1], desired_position[2])
        pose.orientation = to_quaternion(pitch=desired_ea_deg[1], roll=desired_ea_deg[0], yaw=desired_ea_deg[2])

        # NOTE: Here we move with teleport enabled so collisions are ignored
        success = self.__client.simSetObjectPose(object_name, pose, teleport=True)
        if (not success):
            print("Unable to get object. Try updating name")
            return success

        print(f"Moved object '{object_name}' to position: {desired_position} with orientation: {desired_ea_deg}")
    
    '''
    Move airsim object by a predefined path.
    Path format should be: x, y, z, roll_deg, pitch_deg, yaw_deg
    '''
    def move_object_by_path(self, object_name, path_array, sleep_time) -> None:
        for pose in path_array:
            if (not len(pose) == 6):
                print("Invalid pose lenght")
                return 
            
            position = np.array([pose[0], pose[1], pose[2]])
            ea       = np.array([pose[3], pose[4], pose[5]])
            if (not self.move_object_to_position(object_name, position, ea)):
                print("Some error happen during movement")
                break

            time.sleep(sleep_time)

    '''
    Change airsim object texture to one saved on computer given by absolute path
    '''
    def change_object_texture(self, object_name, texture_path) -> bool:
        if (not os.path.exists(texture_path)):
            print(f"File '{texture_path}' not exists")
            return False
        
        res = self.__client.simSetObjectMaterialFromTexture(object_name, texture_path)
        if (not res):
            print(f"Unable to change texture to object: {object_name}")
            return res

        print(f"Changed texture of object '{object_name}' to {texture_path}")
        return res
    

    '''
    Change airsim object segmentation ID. The name input will be a regex function or a unique name.
    Read the following part of the wiki: https://microsoft.github.io/AirSim/image_apis/#segmentation
    '''
    def change_object_segmentation_id(self, object_name, object_new_id, is_regex=False) -> bool:
        res = self.__client.simSetSegmentationObjectID(object_name, object_new_id, is_regex)
        if (not res):
            print(f"Unable to change segmentation id to object: {object_name}")
            return res

        print(f"Change segmentation id of object '{object_name}' to {object_new_id}")
        return res
    

def main(args):

    manager = AirsimEnvironmentManager(args)
    print("Created AirsimEnvironmentManager")

    if(args.sim_weather):
        manager.enable_weather()
        manager.change_weather(airsim.WeatherParameter.Fog, 0.2)

    if (args.move_object):
        object_name = "pipe_long_1"
        path_array = []
        path_array.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        path_array.append([0.0, 0.0, 2.0, 0.0, 0.0, 0.0])
        path_array.append([0.0, 0.0, 4.0, 0.0, 0.0, 0.0])
        path_array.append([0.0, 0.0, 6.0, 0.0, 0.0, 0.0])
        manager.move_object_by_path(object_name, path_array, 1.0)

    if (args.change_segmentation_id):
        object_name_regex = "pipe_[\w]*"
        object_new_id = 22
        manager.change_object_segmentation_id(object_name_regex, object_new_id, True)

    if (args.change_texture):
        texture_path = "/home/aiiacvmllab/Downloads/rusty-metallic-textured-background.jpg"

        object_name = "pipe_long_1"
        manager.change_object_texture(object_name, texture_path)

        object_name = "pipe_elbow_1"
        manager.change_object_texture(object_name, texture_path)


if __name__ == "__main__":
    parser = ArgumentParser(description="Configure Airsim environment")
    parser.add_argument("--sim_weather", action="store_true", default=False)
    parser.add_argument("--move_object", action="store_true", default=False)
    parser.add_argument("--change_texture", action="store_true", default=False)
    parser.add_argument("--change_segmentation_id", action="store_true", default=False)

    args = parser.parse_args()

    main(args)