# ros_tracking
Multiobject Tracking for robots in ROS2. This is designed to be fast, accurate, modular, and reconfigurable. 

This package is under active development as part of my Ph.D. in robotics at UT Austin--if there is a feature you would like to see, please contact me or raise an issue!

# Prerequisites
This assumes you are using Ubuntu 22.04

# Setup
## Clone the repo
## Setting up the CONDA environment
## Download nuScenes data (Optional)
If you want to use the nuScenes dataset for tracking development and evaluation, follow these steps to download the dataset.

1) [Create an account with nuScenes at this link](https://www.nuscenes.org/sign-up).
2) Create a directory to store the data and navigate to it:
```
cd ~
mkdir nuscenes
cd nuscenes
```
4) At the [downloads section of the nuScenes page](https://www.nuscenes.org/nuscenes#download), download the US versions of the Mini, Trainval, and Test Metadata and file blobs.

  NOTE: If using a headless display (e.g. a server), you can use wget to download the files as described [here](https://github.com/nutonomy/nuscenes-devkit/issues/110). An example command format is provided below:
  ```
  wget -O v1.0-mini.tgz "https://s3.amazonaws.com/data.nuscenes.org/public/v1.0/v1.0-mini.tgz?..."
  ```
Upon completion, you should have the following files in your `~/nuscenes` directory:
```
v1.0-mini.tgz
v1.0-test_blobs.tgz
v1.0-test_meta.tgz
v1.0-trainval01_blobs.tgz
v1.0-trainval02_blobs.tgz
v1.0-trainval03_blobs.tgz
v1.0-trainval04_blobs.tgz
v1.0-trainval05_blobs.tgz
v1.0-trainval06_blobs.tgz
v1.0-trainval07_blobs.tgz
v1.0-trainval08_blobs.tgz
v1.0-trainval09_blobs.tgz
v1.0-trainval10_blobs.tgz
v1.0-trainval_meta.tgz
```
5) Extract the blob files. For each `.tgz` file, extract it as follows:
```
tar -xvf v1.0-mini.tgz
tar -xvf v1.0-test_blobs.tgz
tar -xvf v1.0-test_meta.tgz
tar -xvf v1.0-trainval01_blobs.tgz
...
tar -xvf v1.0-trainval_meta.tgz
```

6) Download and extract the megvii detector data:
```
wget https://www.nuscenes.org/data/detection-megvii.zip
unzip detection-megvii.zip -d detection-megvii
```

Upon completion, the directory should have the following structure:
```
~/nuscenes
├── detection-megvii
├── detection-megvii.zip
├── maps
├── samples
├── sweeps
├── v1.0-mini
├── v1.0-test
├── v1.0-trainval
```
At this point, you can remove the .zip and .tgz files if you'd like.

# Usage
## Development using nuScenes data (Optional)
If you want to use the nuScenes dataset and evaluation scripts to develop the tracker, complete the following steps:
1) Convert nuscenes detection and data to ROS2 .mcap files.
2) Re-index and save nuScenes annotations.
By default, the nuScenes ground truth annotations are indexed by split > scene > sample (frame) > annotation (object instance). However, to learn the motion model parameters, the ground truth annotations should be indexed by split > scene > annotation (object instance) > sample (frame) , so that the state of the object can be observed as it progresses through each sample frame. 
3) 
## 
