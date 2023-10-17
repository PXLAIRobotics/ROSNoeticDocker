#!/bin/bash

# Checking if host is aleady part of a swarm
swarm_status=$(docker info  2> /dev/null | grep -E "^[[:space:]]*Swarm:[[:space:]]*.?.?active" \
    | awk -F":" '{ print $2 }' | tr -d [:blank:]) 

if [ "$swarm_status" = "inactive" ]; then
    echo "[STOPPING] THIS HOST ISN'T PART OF A SWARM?!?!"
    exit -1
fi

echo "The overlay network IP address of the container is..."
IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' swarm_master)
echo $IP > docker_container_overlay_network_ip.log
echo $IP
