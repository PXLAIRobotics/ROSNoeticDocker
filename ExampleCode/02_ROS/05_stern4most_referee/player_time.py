import rospy

class PlayerTime():
    def __init__(self, current_round, current_sector, total_time, last_sector_time):
        self.current_round = current_round
        self.current_sector = current_sector
        self.total_time = total_time
        self.last_sector_time = last_sector_time

    def to_string(self):
        return str(self.current_round) + ", " + str(self.current_sector) + ", " + str(self.total_time) + ", " + str(self.last_sector_time)
    def __str__(self):
        return self.to_string()
    def __repr__(self):
        return repr(self.to_string())

    def update(self, sector, start_time):
        if sector-1 != self.current_sector:
            return False

        now = rospy.get_rostime().to_sec()
        self.current_sector = sector
        self.last_sector_time = now - self.total_time - start_time
        self.total_time = now - start_time
        
        return True