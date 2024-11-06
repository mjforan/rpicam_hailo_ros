Enable PCIe Gen 3.0 speed and install hailo drivers:
```
sudo su
echo "dtparam=pciex1_gen=3" >> /boot/firmware/config.txt
apt install -y hailo-all
reboot now
```

Example rpicam-vid stream using ROS publisher post-processing stage:
```
docker run --rm -it --privileged --network=host -v /run/udev:/run/udev -v ./hailo_yolov8_inference_ros.json:/rpicam-apps/assets/hailo_yolov8_inference_ros.json -v ./go2rtc.yaml:/go2rtc.yaml mjforan/rpicam-ros bash
wget https://github.com/AlexxIT/go2rtc/releases/latest/download/go2rtc_linux_arm64
chmod +x go2rtc_linux_arm64
./go2rtc_linux_arm64 --config /go2rtc.yaml &
. /ros_entrypoint.sh
rpicam-vid --vflip --hflip --mode 4608:2592 --width 1200 --height 674 --framerate 56.03 --denoise cdn_off --timeout 0 --nopreview --codec h264 --level 4.2 --libav-format h264 --inline -o udp://127.0.0.1:8099 --post-process-file /rpicam-apps/assets/hailo_yolov8_inference_ros.json --lores-width 640 --lores-height 640 &
ros2 topic echo /detections
```