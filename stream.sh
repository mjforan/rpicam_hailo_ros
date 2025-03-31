#!/usr/bin/env bash

rpicam-vid --mode 1536:864     \
           --width 1536        \
           --height 864        \
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





