#!/bin/bash

# Checking if host is aleady part of a swarm
swarm_status=$(docker info  2> /dev/null | grep -E "^[[:space:]]*Swarm:[[:space:]]*.?.?active" \
    | awk -F":" '{ print $2 }' | tr -d [:blank:]) 

if [ "$swarm_status" = "inactive" ]; then
    echo "[STOPPING] THIS HOST ISN'T PART OF A SWARM?!?!"
    exit -1
fi

docker swarm leave --force
rm ./docker_container_overlay_network_ip.log 2> /dev/null
rm ./docker_swarm_network.log 2> /dev/null
rm ./docker_swarm_master.log 2> /dev/null
rm ./Commands/bin/set_swarm_settings.bash 2> /dev/null
