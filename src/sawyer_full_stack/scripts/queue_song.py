#!/usr/bin/env python

""" Node that listens in to head ar tracker and commands script to pick and place correct node """
import random 
import rospy
import rospkg
import roslaunch
from ar_track_alvar_msgs.msg import AlvarMarkers

import pick_and_place

#!/usr/bin/env python

import rospy
from sawyer_full_stack.srv import QueueSong, QueueSongResponse
from std_msgs.msg import Int64

class QueueSongServiceNode:
    def __init__(self):
        rospy.init_node('queue_audience_song')

        self.seen_markers = set()
        self.used_markers = set([10])   # set of integers of used markers
        self.audience_markers = set(list(range(10, 20)))  # markers that the audience can hold up, differentiate from markers on table TODO: Fill out with the markers we decide are valid
        self.prev_desk_marker = 0 # need to put ar tag 0 down at the table before we run our code

        # Subscribe to the topic that publishes ar pose data
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_marker_callback)

        # Create the service server
        self.service = rospy.Service('queue_audience_song', QueueSong, self.handle_queue_audience_song)

    def ar_marker_callback(self, data):
        # Callback function to process incoming data from the topic
        print(data)
        self.seen_markers = set([int(marker.id) for marker in data.markers] ) # set of currently seen markers

    def get_cube_ar_tag(self, audience_ar_tag):
        return audience_ar_tag - 10

    def handle_queue_audience_song(self, request):
        # Service callback function to handle sum requests
        # Respond with the sum up to the specified start_index
        valid_marker_names = (self.seen_markers - self.used_markers) & self.audience_markers
        if (len(valid_marker_names) > 0):
            marker_to_queue = random.choice(list(valid_marker_names))
        else:
            print("No song request seen")

        print(valid_marker_names)
        desk_marker = self.get_cube_ar_tag(marker_to_queue)
        # switch from head to arm camera
        status = pick_and_place(desk_marker, self.prev_desk_marker)
        # call service to add song to queue
        self.prev_desk_marker = desk_marker

        # TODO: make this message
        response = QueueSongResponse()
        response.success = status 
        return response

if __name__ == '__main__':
    try:
        service_node = QueueSongServiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



# # ------ CHANGING THIS TO BE A SERVICE -------
# class QueueSong():
#     def __init__(self):
#         self.used_markers = set([10])   # set of integers of used markers
#         self.audience_markers = set(list(range(10, 20)))  # markers that the audience can hold up, differentiate from markers on table TODO: Fill out with the markers we decide are valid
#         self.prev_desk_marker = 0 # need to put ar tag 0 down at the table before we run our code

#     def get_cube_ar_tag(self, audience_ar_tag):
#         return audience_ar_tag - 10

#     def ar_marker_callback(self, data):
#         print(data)
#         current_marker_names = set([int(marker.id) for marker in data.markers] ) # set of currently seen markers
#         valid_marker_names = (current_marker_names - self.used_markers) & self.audience_markers
#         if (len(valid_marker_names) > 0):
#             marker_to_queue = random.choice(list(valid_marker_names))
#         else:
#             print("No song request seen")
#         print(valid_marker_names)
#         desk_marker = self.get_cube_ar_tag(marker_to_queue)
#         # switch from head to arm camera
#         pick_and_place(desk_marker, self.prev_desk_marker)
#         # call service to add song to queue
#         self.prev_desk_marker = desk_marker



# if __name__ == "__main__":
#     try:
#         allow_execution = True
#         rospy.init_node('queue_audience_song', anonymous =  True)
#         queue_helper = QueueSong()

#         rospy.Subscriber("/ar_pose_marker", AlvarMarkers, queue_helper.ar_marker_callback)

#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass