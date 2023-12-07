#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import time
from Queue import Queue  # For Python 2
from spotify_api import add_song_to_queue
from song_queue.srv import NewSongRequest
from pick_and_place import move_cube, move_cube_now_playing
# If you are using Python 3, use "from queue import Queue" instead

class QueueProcessorNode:
    def __init__(self):
        rospy.init_node('queue_processor_node', anonymous=True)
        
        self.queue = []
        self.processing_time = rospy.get_param('~processing_time', 1.0)  # Default processing time is 1 second
        self.loop_rate = rospy.Rate(10)  # 10 Hz
        self.pause_process = 1

        rospy.Subscriber('/add_to_queue', NewSongRequest, self.add_to_queue_callback)
        self.subscriber = rospy.Subscriber('/song_changing', bool, self.song_changing_callback)
        self.queue_subscriber = rospy.Subscriber('/add_to_queue', NewSongRequest, self.add_to_queue_callback)
    
    def song_changing_callback(self, msg):
        # Callback function that stores the received message
        self.pause_process = self.pause_process * -1

    def add_to_queue_callback(self, data):
        rospy.loginfo('Adding to queue: %s', data.ar_tag, data.song)
        self.queue = data.data
        self.process_queue(self.queue)

    def process_queue(self):
        while not rospy.is_shutdown() and self.pause_process == 1:
            if len(self.queue) > 0:
                ar_tag, song = self.queue.pop(0)
                add_song_to_queue(song) 
                # self.taken_tags.remove(ar_tag)
                # self.available_tags.append(ar_tag)
                print("MOVING CUBE")
                self.move_cube(int(ar_tag), int(self.prev_ar_tag))
                self.prev_ar_tag = ar_tag

                #time how long move cube takes
                time.sleep(self.processing_time)

            # Check if there is time left, otherwise wait
            self.loop_rate.sleep()

if __name__ == '__main__':
    try:
        node = QueueProcessorNode()
        node.process_queue()
    except rospy.ROSInterruptException:
        pass
