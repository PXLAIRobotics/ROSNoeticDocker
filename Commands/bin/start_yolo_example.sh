#!/bin/bash

# Navigate to the appropriate directory
cd ~/Projects/catkin_ws

# If a tmux session named 'ros_session' exists, kill it
tmux has-session -t ros_session 2>/dev/null

if [ $? != 1 ]; then
  tmux kill-session -t ros_session
fi

# Start a new tmux session in the background
tmux new-session -d -s ros_session

# Play the rosbag
tmux send-keys -t ros_session "rosbag play ~/Data/bags/person.bag -r 0.5" C-m

# Sleep to give some time for rosbag to start
sleep 2

# Split pane and run yolo_example node
tmux split-window -v
tmux send-keys -t ros_session "rosrun yolo_example object_detector.py" C-m

# Sleep to give some time for node to start
sleep 2

# Split pane and echo the topic
tmux split-window -v
tmux send-keys -t ros_session "rostopic echo /detected_objects" C-m

# Attach to the session to view the outputs
tmux attach -t ros_session
