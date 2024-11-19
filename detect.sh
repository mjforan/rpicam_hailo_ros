#! /usr/bin/env bash

# go2rtc will run stream.sh when a user connects to the webpage
go2rtc --config /config/go2rtc.yaml &

# rpicam-vid will be killed when the streaming process takes over, but when it stops we need to start detecting again
while true; do
  if ! pgrep -x "rpicam-vid" > /dev/null; then
    # 26fps determined by max performance of AI accelerator, yuv420 for cheapest encoding since the video is just getting dumped
    rpicam-vid --vflip --hflip --mode 1536:864 --width 640 --height 640 --framerate 26 --timeout 0 --nopreview --codec yuv420 --post-process-file /config/hailo_yolov8_inference_ros.json -o /dev/null &
    # save the PID so the streaming process can kill it
    echo $! > /rpicam.pid
    sleep 1
  fi
done
