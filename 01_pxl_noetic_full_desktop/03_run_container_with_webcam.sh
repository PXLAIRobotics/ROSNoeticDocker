#!/bin/bash

if ! command -v glxinfo &> /dev/null
then
    echo "glxinfo command  not found! Execute \'sudo apt install mesa-utils\' to install it."
    exit
fi

# A webcam is a character device.
if [ ! -c /dev/video0 ]; then
    echo "No webcam found, exiting script."
    exit
fi

vendor=`glxinfo | grep vendor | grep OpenGL | awk '{ print $4 }'`


if [ $vendor == "NVIDIA" ]; then
    docker run -it --rm \
        --name noetic_desktop \
        --hostname noetic_desktop \
        --device /dev/snd \
        --device=/dev/video0:/dev/video0 \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -v `pwd`/../Commands/bin:/home/user/bin \
        -v `pwd`/../ExampleCode:/home/user/ExampleCode \
        -v `pwd`/../Projects/catkin_ws_src:/home/user/Projects/catkin_ws/src \
        -env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --gpus all \
        pxl_noetic_full_desktop:latest \
        bash
else
    docker run --privileged -it --rm \
        --name noetic_desktop \
        --hostname noetic_desktop \
        --volume=/tmp/.X11-unix:/tmp/.X11-unix \
        -v `pwd`/../Commands/bin:/home/user/bin \
        -v `pwd`/../ExampleCode:/home/user/ExampleCode \
        -v `pwd`/../Projects/catkin_ws_src:/home/user/Projects/catkin_ws/src \
        --device=/dev/dri:/dev/dri \
        --env="DISPLAY=$DISPLAY" \
        -e "TERM=xterm-256color" \
        --cap-add SYS_ADMIN --device /dev/fuse \
        --device=/dev/video0:/dev/video0 \
        pxl_noetic_full_desktop:latest \
        bash
fi
