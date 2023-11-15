# ros_tracking

# Setup
## Prerequisites
sudo apt-get install ros-$ROS_DISTRO-rosbag2 ros-$ROS_DISTRO-rosbag2-storage-mcap mcap

pip install nuscenes-devkit 'mcap-protobuf-support>=0.0.8'

## Clone repo

## Install Dependencies
```
cd ~/my_ws/
rosdep install -i --from-path src --rosdistro iron -y
```

## OAK-D (optional)
Set up udev rules
```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```