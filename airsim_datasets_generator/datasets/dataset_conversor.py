#!/usr/bin/env python3

from argparse import ArgumentParser

from dataset_format_immpi  import convert_dataset_to_ImMPI_format
from dataset_format_hdf5   import convert_dataset_to_hdf5_format
from dataset_format_f2nerf import convert_dataset_to_f2nerf_format

if __name__ == "__main__":
    parser = ArgumentParser(description="Convert a dataset to different formats")
    parser.add_argument('--format', type=str, choices=["immpi", "hdf5", "f2nerf"], default="immpi")
    parser.add_argument('--dataset_path', type=str, default="")
    args = parser.parse_args()

    if args.dataset_path == "":
        print("You must specify the dataset path. See --help for more information")
        exit()

    if args.format == "immpi":
        convert_dataset_to_ImMPI_format(args.dataset_path)
    elif args.format == "hdf5":
        convert_dataset_to_hdf5_format(args.dataset_path)
    elif args.format == "f2nerf":
        convert_dataset_to_f2nerf_format(args.dataset_path)
    else:
        print("The format {} is not implemented yet".format(args.format))
        exit()
