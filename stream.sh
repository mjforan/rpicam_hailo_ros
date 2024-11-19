#! /usr/bin/env bash

# stop the rpicam-vid running from detection.sh
kill -9 $(<"/rpicam.pid")
exec rpicam-vid --vflip --hflip --mode 1536:864 --width 1152 --height 648 --framerate 30 --timeout 0 --nopreview --codec h264 --level 4.1 --libav-format h264 --post-process-file /config/hailo_yolov8_inference_ros.json -o - &
