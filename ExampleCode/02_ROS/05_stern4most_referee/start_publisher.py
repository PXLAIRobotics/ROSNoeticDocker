#!/usr/bin/env python
import os
import rospy
import sys
from std_msgs.msg import String

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("missing argument")
        exit(0)
    
    # number of 
    target_connections = int(sys.argv[1]) + 1
    print(str(target_connections))

    publisherName = "referee_" + str(os.getpid())
    topicName     = "game_on"

    try:
        rospy.init_node(publisherName, anonymous=False)        
        publisher = rospy.Publisher(topicName, String, queue_size=1)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            connections = publisher.get_num_connections()
            connections_left = target_connections - connections
            wait_msg = 'Waiting for ' + str(connections_left) + ' more players and/or the referee service'

            rospy.loginfo('Connections: %d', connections)
            rospy.loginfo(wait_msg)
            publisher.publish(wait_msg)

            if connections == target_connections:
                for i in range(5, 0, -1):
                    publisher.publish(str(i))
                    rate.sleep()
                publisher.publish('Start')
                rospy.loginfo('published')
                break
            rospy.sleep(5)
            #rate.sleep()

            
    except rospy.ROSInterruptException:
        print("An interrupt occurred.")