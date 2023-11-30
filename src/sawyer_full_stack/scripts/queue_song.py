#!/usr/bin/env python

""" Node that listens in to head ar tracker and commands script to pick and place correct node """
import random 
import rospy
import rospkg
import roslaunch
from ar_track_alvar_msgs.msg import AlvarMarkers

import pick_and_place

class QueueSong():
    def __init__(self):
        self.used_markers = set()   # set of integers of used markers
        self.audience_markers = set([16])  # markers that the audience can hold up, differentiate from markers on table TODO: Fill out with the markers we decide are valid
        self.prev_marker = 0 # need to put ar tag 0 down at the table before we run our code


    def ar_marker_callback(self, data):
        print(data)
        current_marker_names = set([int(marker.id) for marker in data.markers] ) # set of currently seen markers
        valid_marker_names = (current_marker_names - self.used_markers) & self.audience_markers
        if (len(valid_marker_names) > 0):
            marker_to_queue = random.choice(list(valid_marker_names))
        print(valid_marker_names)
        # switch from head to arm camera
        pick_and_place(marker_to_queue, self.prev_marker)
        # call service to add song to queue
        self.prev_marker = marker_to_queue



if __name__ == "__main__":
    try:
        allow_execution = True
        rospy.init_node('queue_audience_song', anonymous =  True)
        queue_helper = QueueSong()

        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, queue_helper.ar_marker_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass