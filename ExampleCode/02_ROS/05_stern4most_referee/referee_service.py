#!/usr/bin/env python 

import rospy 
import sys
import os
import operator
import optparse
from referee.srv import * 
from threading import Thread
from threading import Lock
from std_msgs.msg import String
from player_time import PlayerTime
from enum import Enum
from functools import cmp_to_key

def comp(first, second):
    first = first[1]
    second = second[1]
    if first.current_round == second.current_round:
        if first.current_sector == second.current_sector:
            if first.total_time < second.total_time:
                return -1
            else:
                return 1
        else:
            if first.current_sector > second.current_sector:
                return -1
            else:
                return 1
    else:
        if first.current_round > second.current_round:
            return -1
        else:
            return 1

class Status(Enum):
    IN_PROGRESS = 1
    WRONG_SECTOR = 2
    FINISHED = 3

class Referee():
    def __init__(self, total_rounds, total_sectors):
        self.total_rounds = total_rounds
        self.total_sectors = total_sectors
        self.start_time = 0
        self.player_times = dict()
        self.lock = Lock()

        rospy.init_node('referee_server') 
    
    def run(self): 
        # separate thread for the rankings publisher
        try:
            thread = Thread(target=self.publish_rank, args=())
            thread.start()
        except:
            rospy.logerr("Error: unable to start thread")

        topic_name = "game_on"
        
        subscriber = rospy.Subscriber(topic_name, String, self.start_callback)
        s = rospy.Service('status_update', SectorUpdate, self.handle_status_update) 

        rospy.loginfo("Status Update Server Ready")

        rospy.spin()

    # set start time to GO
    def start_callback(self, data):
        rospy.loginfo(data.data)
        if data.data == 'Start':
            self.start_time = rospy.get_rostime().to_sec()
            rospy.loginfo('Start time: ' + str(self.start_time))

    def update_player_times(self, name, sector):
        status = Status.IN_PROGRESS

        with self.lock:
            # update existing player
            if name in self.player_times:
                player_time = self.player_times[name]

                if not player_time.update(sector, self.start_time):
                    rospy.loginfo("%s WRONG SECTOR" % (name))
                    status = Status.WRONG_SECTOR

                if sector == self.total_sectors and player_time.current_round == self.total_rounds:
                    rospy.loginfo("%s FINISHED" % (name))
                    status = Status.FINISHED
                elif sector == self.total_sectors:
                    player_time.current_round += 1
                    player_time.current_sector = 0
                    rospy.loginfo("%s entered round %d" % (name, player_time.current_round))
                
                current_round = player_time.current_round

            # add new player (no sector info sent yet)
            else: 
                if sector != 1:
                    rospy.loginfo("%s WRONG SECTOR" % (name))
                    sector = 0
                    status = Status.WRONG_SECTOR
                    total_time = 0
                else:
                    now = rospy.get_rostime().to_sec() 
                    total_time = now - self.start_time
                
                current_round = 1
                self.player_times[name] = PlayerTime(current_round, sector, total_time, total_time)

        if status == Status.IN_PROGRESS:
            return 'Round: ' + str(current_round) + ' - Sector: ' + str(sector)
        else:
            return status.name

    def handle_status_update(self, req): 
        rospy.loginfo("Received: %s from %s" % (req.sector, req.name))

        status = self.update_player_times(req.name, int(req.sector))

        player_time = self.player_times[req.name]

        rospy.loginfo("Returning (%f, %f, %s)" % (player_time.total_time, player_time.last_sector_time, status)) 
        return SectorUpdateResponse(player_time.total_time, player_time.last_sector_time, status)  

    # rank publisher
    def publish_rank(self):
        publisher_name = "ranking_" + str(os.getpid())
        topic_name     = "ranking"

        try:       
            publisher = rospy.Publisher(topic_name, String, queue_size=1) 

            rate = rospy.Rate(0.1)
            while not rospy.is_shutdown():
                if(len(self.player_times) > 0):
                    # sort player_times on the total_time for each player
                    with self.lock:
                        rankings = sorted(self.player_times.items(), key=cmp_to_key(comp))

                    # send the list of sorted tuples (playerName, (total_time, last_sector_time, current_round))
                    rospy.loginfo("Rank Publisher Sending: " + str(rankings))
                    publisher.publish(str(rankings))
                    rospy.loginfo("Sent") 

                rate.sleep()

        except rospy.ROSInterruptException:
            rospy.logerr("An interrupt occurred.")

def parse_arguments():  
    parser = optparse.OptionParser()

    parser.add_option('-r', '--rounds',
        action="store", dest="rounds",
        help="number of rounds", default="2")
    parser.add_option('-s', '--sectors',
        action="store", dest="sectors",
        help="number of sectors", default="5")

    options, args = parser.parse_args()

#    global rounds, sectors
    total_rounds = int(options.rounds)
    total_sectors = int(options.sectors)

    print("Rounds: " + str(total_rounds))
    print("Sectors: " + str(total_sectors))

    return total_rounds, total_sectors

if __name__ == "__main__": 
    total_rounds, total_sectors = parse_arguments()

    referee = Referee(total_rounds, total_sectors)
    referee.run()