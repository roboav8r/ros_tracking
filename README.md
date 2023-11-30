# ros_tracking

# Setup
## Prerequisites
Create a ros2 environment
```
https://robostack.github.io/GettingStarted.html
```
<!-- sudo apt-get install ros-$ROS_DISTRO-rosbag2 ros-$ROS_DISTRO-rosbag2-storage-mcap mcap -->

python3 -m pip install nuscenes-devkit 'mcap-protobuf-support>=0.0.8' pandas

## Clone repo

## Install Dependencies
```
cd ~/my_ws/
rosdep install -i --from-path src --rosdistro iron -y
```
### GTSAM
```
cd ~
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build
cd build
cmake .. -DGTSAM_BUILD_PYTHON=1
make
make python-install # this will need to be done for each virtual/conda environment you use
sudo apt-get install ros-iron-gtsam
```

## OAK-D (optional)
Set up udev rules
```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```