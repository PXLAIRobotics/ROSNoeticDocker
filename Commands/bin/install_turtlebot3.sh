#!/bin/bash

(sudo chown user:user /home/user/Ptojects/catkin_ws/src)
(cd /home/user/Projects/catkin_ws/src/; git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git)
(cd /home/user/Projects/catkin_ws/src/; git clone https://github.com/ROBOTIS-GIT/turtlebot3.git)
(cd /home/user/Projects/catkin_ws/src/; git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git)
(cd /home/user/Projects/catkin_ws; catkin_make)
