#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import rospkg
from song_queue.srv import SongEnding
from song_queue.srv import NewSongRequest
from spotify_api import add_song_to_queue
from sawyer_full_stack.scripts import pick_and_place

class DecisionMaker:
    def __init__(self, tags, prev_ar_tag = -1):
        self.available_tags = tags
        self.taken_tags = []
        self.visible_tags = tags
        self.queue = []
        self.prev_ar_tag = prev_ar_tag
    #Callback for when song is added to Spotify queue
    def handle_change_song(self, req):
        #TODO: fix this
        if len(queue) > 0:
            ar_tag, song = queue.pop(0)
            add_song_to_queue(song) 
            self.taken_tags.remove(ar_tag)
            self.available_tags.append(ar_tag)
            pick_and_place(ar_tag, self.prev_ar_tag)
            self.prev_ar_tag = ar_tag

        else:
            for i in range(len(queue)):
                if queue[i][0] == msg.ar_tag:
                    ar_tag, song = queue.pop(0)
                    add_song_to_queue(song)
                    return
    #Callback for when song is added to local queue (aka human adds ar tag: song pair)
    def handle_song_request(self, req):
        self.queue.append([req.ar_tag, req.song])
        self.available_tags.remove(req.ar_tags)
        self.visible_tags.remove(req.ar_tags)
    def work(self):
        rospy.init_node('decision_maker')
        song_requeset_service = rospy.Service('song_request_channel', NewSongRequest, self.handle_song_request)
        change_song_service = rospy.Service('change_song_service', SongEnding, self.handle_change_song)
        rospy.spin()



if __name__ == "__main__":
    try:
        DecisionMaker([11], 9)
    except rospy.ROSInterruptException as e:
        print(e)
        pass