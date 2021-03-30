#!/usr/bin/env python3


import rospy
import math
import sys

from sensor_msgs.msg import LaserScan


class LaserListener():
    def __init__(self, output="overview"):
        self.output = output
        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.callback_scan)

    def callback_scan(self, data):
        if self.output == "overview":
            print("Topic /scan has data from: %s rad or %s°" % (data.angle_min, math.degrees(data.angle_min)))
            print("                       to: %s rad or %s°" % (data.angle_max, math.degrees(data.angle_max)))
            print("       with increments of: %s rad or %s°" % (data.angle_increment, math.degrees(data.angle_increment)))
            print("      and a range between: %s and %s" % (data.range_min, data.range_max))
            print("                 in total: %s values" % (len(data.ranges)))
            print()
        elif self.output == "data":
            index = 0
            object_found = False
            for value in data.ranges:
                if not math.isinf(value):
                    if object_found == False:
                        object_found = True
                    current_angle = data.angle_min + (data.angle_increment * index)
                    print("Found an object at %s°, distance: %s" % (math.degrees(current_angle), value))
                index+=1

            if object_found == False:
                print("No object(s) found in range.")

def usage():
    return "Usage:\n    %s overview\n Or:\n    %s data" % (sys.argv[0], sys.argv[0])


if __name__ == "__main__":
    if len(sys.argv) == 2:
        argument = str(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)

    rospy.init_node("laser_listener")
    
    if argument == "overview":
        LaserListener()
    else:
        LaserListener("data")
        
    rospy.spin()
