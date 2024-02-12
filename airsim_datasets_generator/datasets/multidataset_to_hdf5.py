#!/usr/bin/env python3

from dataset_format_hdf5 import convert_multiples_dataset_to_hdf5_format

datasets_folders = ["/home/mmontes/Documents/AUTH/datasets/ue4_datasets/datas/dataset_2023-06-03_01:29:34", 
                    "/home/mmontes/Documents/AUTH/datasets/ue4_datasets/datas/dataset_2023-06-03_01:35:57", 
                    "/home/mmontes/Documents/AUTH/datasets/ue4_datasets/datas/dataset_2023-06-03_01:40:41", 
                    "/home/mmontes/Documents/AUTH/datasets/ue4_datasets/datas/dataset_2023-06-03_01:41:47", 
                    ]
convert_multiples_dataset_to_hdf5_format(datasets_folders, "/home/mmontes/Documents/AUTH/datasets/multiview2novelview/ue4")