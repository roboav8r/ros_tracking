# ros_tracking
Multiobject Tracking for robots in ROS2. This is designed to be fast, accurate, modular, reconfigurable, and capable of running on embedded hardware. 

This package is under active development as part of my Ph.D. in robotics at UT Austin--if there is a feature you would like to see, please contact me or raise an issue!

# Prerequisites
This assumes you are using Ubuntu 22.04 with ROS2 and `conda` installed. This was developed using ROS2 Iron, installed from binary using the instructions [here](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html#). 

Note: Make sure that you deactivate any virtual environments before installing ROS2.

## Tracing (Optional) 
If you want to perform tracing analysis for performance or development purposes, you can verify that tracing is enabled by executing `ros2 run tracetools status` at a properly sourced terminal. Then, install `ros2_tracing`:
```
sudo apt-get install ros-iron-ros2trace
```
# Setup
## Clone the repo
```
cd ~
mkdir -p tracking_ws/src && cd tracking_ws/src
git clone https://github.com/roboav8r/ros_tracking.git
cd tracking_ws/src
rosdep install -i --from-path src --rosdistro iron -y
```
## Setting up the Conda environment
Note: I use [`mamba`](https://github.com/conda-forge/miniforge) as a personal preference, but it is a drop-in replacement to `conda`; either `mamba` or `conda` will work for these commands.
```
mamba env create -f environment.yml
mamba activate ros_tracking
```

## Download nuScenes data (Optional)
If you want to use the nuScenes dataset for tracking development and evaluation, follow these steps to download the dataset.

1) [Create an account with nuScenes at this link](https://www.nuscenes.org/sign-up).
2) Create a directory to store the data and navigate to it:
```
cd ~
mkdir nuscenes
cd nuscenes
```
4) At the [downloads section of the nuScenes page](https://www.nuscenes.org/nuscenes#download), download the US versions of the Mini, Trainval, and Test Metadata and file blobs. Also, download the Map expansion v1.3.

  NOTE: If using a headless display (e.g. a server), you can use wget to download the files as described [here](https://github.com/nutonomy/nuscenes-devkit/issues/110). An example command format is provided below:
  ```
  wget -O v1.0-mini.tgz "https://s3.amazonaws.com/data.nuscenes.org/public/v1.0/v1.0-mini.tgz?..."
  ```
Upon completion, you should have the following files in your `~/nuscenes` directory:
```
nuScenes-map-expansion-v1.3.zip
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
6) Unzip the map expansion data into the `maps` folder:
```
unzip nuScenes-map-expansion-v1.3.zip -d maps
```

7) Download and unzip the megvii detector:
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
1) Convert nuscenes detection and data to ROS2 .mcap files. In the `ros_tracking` directory:
```
python3 scripts/nuscenes_to_mcap.py # Converts mini_train by default
python3 scripts/nuscenes_to_mcap.py -s mini_val # Converts the mini_val split
python3 scripts/nuscenes_to_mcap.py -d v1.0-trainval -s train # Converts the training split from the main dataset
python3 scripts/nuscenes_to_mcap.py -d v1.0-trainval -s val # Converts the validation split from the main dataset
python3 scripts/nuscenes_to_mcap.py -d v1.0-test -s test # Converts the test split from the main dataset
```
2) Re-index and save nuScenes annotations.
By default, the nuScenes ground truth annotations are indexed by split > scene > sample (frame) > annotation (object instance). However, to learn the motion model parameters, the ground truth annotations should be indexed by split > scene > annotation (object instance) > sample (frame) , so that the state of the object can be observed as it progresses through each sample frame. 

# Acknowledgements
The nuScenes `.mcap` conversion script is a modified version of the original, available [here](https://github.com/foxglove/nuscenes2mcap). While the original Foxglove version uses protobuf serialization, the [included file](scripts/nuscenes_to_mcap.py) uses Foxglove's ROS2 serialization, with the same datatypes. 


