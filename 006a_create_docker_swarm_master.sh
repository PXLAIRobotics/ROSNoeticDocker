#!/bin/bash

# Checking if host is aleady part of a swarm
swarm_status=$(docker info  2> /dev/null | grep -E "^[[:space:]]*Swarm:[[:space:]]*.?.?active" \
    | awk -F":" '{ print $2 }' | tr -d [:blank:]) 

if [ "$swarm_status" = "active" ]; then
    echo "[STOPPING] THIS HOST IS ALREADY PART OF A SWARM?!?!"
    exit -1
fi

if [ $# -ne 1 ] # One argument (the IP) is needed.
then
    echo "Incorrect number of arguments"
    echo "Usage: "
    echo -e "   $0 IP4.in.dot.format"
    echo
    echo "Possible IPs: "
    ip -br a | grep UP | awk '{ print "   -> "  $1 " is listening on: "  $3}'
    exit -1
elif [[ $1 =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
    echo "Supplied correct IP address format."
else
    echo "Supplied incorrect IP address format! ($1)"
    echo
    echo "Possible IPs: "
    ip -br a | grep UP | awk '{ print "   -> "  $1 " is listening on: "  $3}'
    exit -1
fi

ip=$1

# Make a Docker swarm and make this host its master.
if ! docker swarm init --advertise-addr $ip > docker_swarm_master.log; then
    echo "[STOPPING] COULDN'T CREATE A SWARM!"
    exit -1
else
    echo "SWARM SUCCESSFULLY CREATED."
fi

# Create an attachable overlay network
if ! docker network create -d overlay --attachable pxl_ros_noetic_overlay_network > docker_swarm_network.log; then
    echo "[STOPPING] COULDN'T CREATE AN OVERLAY NETWORK!"
    exit -1
else
    echo "OVERLAY NETWORK SUCCESSFULLY CREATED."
fi

# Show the output (This will include the join command.)
echo "SHOWING JOIN COMMAND:"
cat ./docker_swarm_master.log
