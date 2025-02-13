# rpicam_ros

The goal for this project is to enable the lowest latency possible for web streaming and Hailo AI inference + ROS 2 publishing **simultaneously**.
The standard way of doing things (post processing stages in `rpicam-vid`) adds latency to the final output.
Instead, `rpicam-vid` output is mirrored to two v4l2 loopback devices. One device is used as an input to the live stream,
while the other is opened by the AI processor.

# Installation
For Raspberry Pi 5 only.

Enable PCIe Gen 3.0 speed, install hailo drivers and loopback video device:
```
sudo su
echo "dtparam=pciex1_gen=3" >> /boot/firmware/config.txt
apt install -y hailo-all v4l2loopback-utils
echo v4l2loopback >> /etc/modules
echo "options v4l2loopback devices=2 video_nr=50,51" > /etc/modprobe.d/v4l2loopback.conf
reboot now
```

# Troubleshooting
Hailo driver mismatch: The driver on the host must match the version in the container. If you recently upgraded on the host, make sure to reboot.