#!/usr/bin/env bash

# global shutter cam: 1456:1088
# pi cam 3: 1536:864
LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu rpicam-vid --mode 1456:1088    \
           --width 1456        \
           --height 1088       \
           --framerate 30      \
           --timeout 0         \
           --nopreview         \
           --codec h264        \
           --level 4.1         \
           --libav-format h264 \
           --flush             \
           --denoise cdn_off   \
           --profile baseline  \
           --low-latency 1     \
           --output -          \
    | ffmpeg -i - -f v4l2 -codec copy /dev/video50 -f v4l2 -codec copy /dev/video51 2>/dev/null &

go2rtc --config /config/go2rtc.yaml
