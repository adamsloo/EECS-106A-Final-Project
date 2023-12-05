#!/usr/bin/env python

import random 
import rospy
import rospkg
import roslaunch
from ar_track_alvar_msgs.msg import AlvarMarkers

import rospy
from std_msgs.msg import Int64
# Node that listens in to head ar tracker and commands script to pick and place correct node 
class AudienceTrackingSubNode:
    def __init__(self):
        rospy.init_node('audience_tracking')

        self.seen_markers = set() # All the markers the camera 
        self.used_markers = set()   # set of integers of used markers
        self.audience_markers = set([15])  # markers that the audience can hold up, differentiate from markers on table TODO: Fill out with the markers we decide are valid
        self.prev_desk_marker = 0 # need to put ar tag 0 down at the table before we run our code

        # Subscribe to the topic that publishes ar pose data
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_marker_callback)

    def ar_marker_callback(self, data):
        # Callback function to process incoming data from the topic
        self.seen_markers = set([int(marker.id) for marker in data.markers] ) # set of currently seen markers
        self.handle_queue_audience_song()

    def get_cube_ar_tag(self, audience_ar_tag):
        return audience_ar_tag - 10

    def handle_queue_audience_song(self):
        # Service callback function to handle sum requests
        # Respond with the sum up to the specified start_index
        valid_marker_names = (self.seen_markers - self.used_markers) & self.audience_markers
        if (len(valid_marker_names) > 0):
            marker_to_queue = random.choice(list(valid_marker_names))
            print(marker_to_queue)
            self.used_markers.add(marker_to_queue)
            desk_marker = self.get_cube_ar_tag(marker_to_queue)

            # Call pick and place

            self.prev_desk_marker = desk_marker

        else:
            print("No song request seen")

        # switch from head to arm camera
        #status = pick_and_place(desk_marker, self.prev_desk_marker)
        # call service to add song to queue

        # TODO: make this message
        #response = QueueSongResponse()
        #response.success = status 
        #return response

if __name__ == '__main__':
    try:
        audience_tracking = AudienceTrackingSubNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass