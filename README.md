# ros_tracking

# Setup

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