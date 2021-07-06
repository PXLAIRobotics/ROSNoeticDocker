#!/bin/bash

if ! command -v glxinfo &> /dev/null
then
    echo "glxinfo command  not found! Execute \'sudo apt install mesa-utils\' to install it."
    exit
fi

vendor=`glxinfo | grep vendor | grep OpenGL | awk '{ print $4 }'`


if [ $vendor == "NVIDIA" ]; then
    docker run -it --rm \
        --name noetic_desktop \
        --hostname noetic_desktop \
        --device /dev/snd \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -v `pwd`/../Commands/bin:/home/user/bin \
        -v `pwd`/../ExampleCode:/home/user/ExampleCode \
        -v `pwd`/../Projects/catkin_ws_src:/home/user/Projects/catkin_ws/src \
        -v `pwd`/../Data:/home/user/Data  \
        -env="XAUTHORITY=$XAUTH" \
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
	-v `pwd`/../Data:/home/user/Data \
        --device=/dev/dri:/dev/dri \
        --env="DISPLAY=$DISPLAY" \
        -e "TERM=xterm-256color" \
        --cap-add SYS_ADMIN --device /dev/fuse \
        pxl_noetic_full_desktop:latest \
        bash
fi
