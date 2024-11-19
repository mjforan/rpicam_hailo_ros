FROM debian:12

# Keep packages in build cache to speed rebuilding
RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

##########################
##  Build ROS 2
##########################
ARG ROS_DISTRO=jazzy

# Install python3-vsctools here and ignore in rosdep install - Debian package has slightly different name
# Downgrade libssl slightly to fix dependency issue
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
     apt update && \
     apt install -y vcstool python3-rosdep2 python3-vcstools colcon build-essential && \
     apt install -y --allow-downgrades libssl3=3.0.14-1~deb12u2
WORKDIR /ros2_ws
# It would be nice to ADD these repos so Docker caches them
RUN mkdir src && vcs import --input https://raw.githubusercontent.com/ros2/ros2/$ROS_DISTRO/ros2.repos src
ADD https://github.com/ros-perception/vision_msgs.git#ros2 /ros2_ws/src/ros-perception/vision_msgs
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
     apt upgrade -y && rosdep update && \
     rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool"
RUN colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to geometry2 launch launch_ros rcl rmw_cyclonedds_cpp rmw_implementation ros2cli_common_extensions image_common vision_msgs
COPY ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]

##########################
##  Build libcamera
##########################

# Add raspberry pi sources for libcamera and hailo-all
RUN apt update && apt install -y curl gpg && \
    curl -fsSL http://archive.raspberrypi.com/debian/raspberrypi.gpg.key | gpg --dearmor -o /usr/share/keyrings/raspberrypi.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/raspberrypi.gpg] http://archive.raspberrypi.com/debian/ bookworm main" > /etc/apt/sources.list.d/raspi.list

# https://www.raspberrypi.com/documentation/computers/camera_software.html#build-libcamera-and-rpicam-apps

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt update && apt upgrade -y && apt install -y libcamera-dev libjpeg-dev libtiff5-dev libpng-dev libavdevice-dev \
    libboost-program-options-dev libdrm-dev libexif-dev meson ninja-build libopencv-dev hailo-all wget

ADD --keep-git-dir https://github.com/raspberrypi/rpicam-apps.git /rpicam-apps
WORKDIR /rpicam-apps

COPY build_ros.patch /build_ros.patch
RUN git apply /build_ros.patch
COPY object_detect_publish_ros_stage.cpp /rpicam-apps/post_processing_stages/object_detect_publish_ros_stage.cpp
RUN . /ros2_ws/install/setup.sh && meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=disabled -Denable_qt=disabled -Denable_opencv=enabled -Denable_ros=enabled -Denable_tflite=disabled -Denable_hailo=enabled
RUN . /ros2_ws/install/setup.sh && meson compile -C build && meson install -C build
# Update libraries so the new post-processing stage is visible
RUN ldconfig

# TODO statically compile https://github.com/ros2/ament_cmake_ros/blob/3f421a0bafa8de0caae1ae2baefc5b982bf4f81a/ament_cmake_ros/cmake/build_shared_libs.cmake#L15-L20 and only copy binaries to output image

ADD --chmod=755 https://github.com/AlexxIT/go2rtc/releases/latest/download/go2rtc_linux_arm64 /usr/local/bin/go2rtc
RUN mkfifo /video_pipe
RUN apt install -y socat

COPY ./stream.sh /stream.sh
COPY ./detect.sh /detect.sh
