# Airsim Datasets Generator

In this repository you will find a series of tools to generate computer vision datasets using AirSim and visualize them. 

## Dependencies
```
pip3 install scipy pynput
```

## Airsim Python API Installation

To simplify the process, the python API has been isolated in the following [repo](https://github.com/mgrova/airsim_python_api). Here are the instructions on how to install it.

## Airsim Computer Vision Configuration

By default, AirSim reads the configuration file from the file: *'/home/$USER/Documents/AirSim/settings.json'*. To easy generate the computer vision dataset the 'SimMode' parameter should be "ComputerVision".
Adding elements to the 'CaptureSettings' array, we will select which cameras we want to simulate and configure it. To get more information take a look to the [official API](https://microsoft.github.io/AirSim/image_apis/#changing-resolution-and-camera-parameters).

```
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
  "SettingsVersion": 1.2,
  "SimMode": "ComputerVision",
  "CameraDefaults": {
      "CaptureSettings": [
        {
          "ImageType": 0,
          "Width": 1280,
          "Height": 720,
          "FOV_Degrees": 45,
          "AutoExposureSpeed": 100,
          "MotionBlurAmount": 0
        },
        {
          "ImageType": 5,
          "Width": 1280,
          "Height": 720,
          "FOV_Degrees": 45,
          "AutoExposureSpeed": 100,
          "MotionBlurAmount": 0
        }
    ]
  }
}
```


## Usage


The most relevant scripts of this tool are described below. All of them have implemented the '--help' argument to know how to use. Before running some of the simulation managers with Airsim, you must run the simulation in Unreal.

* **airsim_dataset_manager.py**: Main tool to take images from AirSim (must take the same images that were defined in the configuration file). Regarding the camera movement, it can be done through the keyboard or through a text file (see input arguments of the script). The poses are defined with respect to the ENU reference system on the coordinate of 'PlayerStart'.

* **airsim_environment_manager.py**: This tool enables to you to configure and modify the environment. If you have some problems with some objects not moving or modifying. It is convenient to change its name, that way the "Unreal database" will be updated.

* **utils/trajectory_interpolator.py**: Linearly interpolates the position and orientation to extend the poses given by a text file to the desired length.

* **utils/trajectory_visualizer.py**: Allows to visualize in 3D a trajectory given as an input its path.

In addition, there is a large subset of tools for visualizing and converting datasets to the models format evaluated during the research. But for the sake of simplicity, we will not go into it here.

## Dataset format

Below is the format of the dataset that is generated when the *--save_dataset* argument is passed to the script [airsim_dataset_manager](airsim_dataset_manager.py).

### Folders distribution

The following is the distribution of folders for dataset generated:

```
dataset_<current_date>
├── intrinsics.txt
├── poses.txt
├── rgb
│   ├── rgb_*.png
    └── ...
└── segmentation
    ├── segmentation_0000.png
    └── ...
└── depth
    ├── depth_*.png
    └── ...
```

### Intrinsic parameters file (intrinsics.txt)

It will contain the a 3x3 matrix with the intrisics parameters of the simulated camera:

```
#[fx 0 cx; 0 fy cy; 0 0 1]
641.07   0.0  640.0
  0.0  641.07 360.0
  0.0    0.0    1.0
```

### Extrinsics parameters file (poses.txt)

It will contain the extrinsics parameters of the simulated camera respect of the coordinate system defined by 'PlayerStart' in Unreal Engine.

```
#id x y z qw qx qy qz
0000 0.0 0.0 0.0 0.0 0.0 0.0
```