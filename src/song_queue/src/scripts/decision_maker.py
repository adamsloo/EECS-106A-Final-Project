#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String
import rospkg
from song_queue.srv import SongEnding
from song_queue.srv import NewSongRequest
from spotify_api import add_song_to_queue
from pick_and_place import move_cube, move_cube_now_playing
from song_queue.srv import MoveCubeRequest
from remaining_time import get_currently_playing

class DecisionMaker:
    def __init__(self, tags, prev_ar_tag = -1):
        self.available_tags = tags
        self.taken_tags = []
        self.visible_tags = tags
        self.queue = []
        self.now_playing_queue = []
        self.prev_ar_tag = prev_ar_tag
        self.song_change_soon = False
        self.next_playing = None

    #Callback for when song is added to Spotify queue
    def handle_change_song(self, req):
        # pub = rospy.Publisher('/song_changing', bool, queue_size=10)
        # pub.publish(True)
        self.song_change_soon = True
        print("SONG CHANGE")
        print(self.now_playing_queue)
        if len(self.now_playing_queue) > 0:
            ar_tag, song = self.now_playing_queue.pop(0)
            print("PUSHING CUBE")
            try:
                moving = self.move_cube(int(ar_tag), -1)
            except:
                print("error handle change")
        self.song_change_soon = False
        process_queue()

        # #TODO: fix this
        # if len(self.queue) > 0:
        #     ar_tag, song = self.queue.pop(0)
        #     add_song_to_queue(song) 
        #     # self.taken_tags.remove(ar_tag)
        #     # self.available_tags.append(ar_tag)
        #     print("MOVING CUBE")
        #     self.move_cube(int(ar_tag), int(self.prev_ar_tag))
        #     self.prev_ar_tag = ar_tag

        # else:
        #     for i in range(len(queue)):
        #         if queue[i][0] == msg.ar_tag:
        #             ar_tag, song = queue.pop(0)
        #             add_song_to_queue(song)
        return True
    #Callback for when song is added to local queue (aka human adds ar tag: song pair)
    def handle_song_request(self, req):
        # pub = rospy.Publisher('/add_to_queue', NewSongRequest, queue_size=20)
        # pub.publish(req)
        print(req.ar_tag, req.song)
        self.queue.append([req.ar_tag, req.song])
        self.now_playing_queue.append([req.ar_tag, req.song])
        self.process_queue()
        # # self.available_tags.remove(req.ar_tag)
        # # self.visible_tags.remove(req.ar_tag)
        return True
    
    def process_queue(self):
        time_left, _ = get_currently_playing()
        #while not rospy.is_shutdown() and self.song_change_soon is False:
        start_time = time.time()
        while len(self.queue) > 0:
            if time.time() - start_time > time_left - 30:
                print("we don't have enough time")
                return False
            print("DA QUEUE")
            print(self.queue)
            ar_tag, song = self.queue.pop(0)
            add_song_to_queue(song) 
            # self.taken_tags.remove(ar_tag)
            # self.available_tags.append(ar_tag)
            print("MOVING CUBE")
            try:
                moving = self.move_cube(int(ar_tag), int(self.prev_ar_tag))
            except:
                print("error u already knowww")
            self.prev_ar_tag = ar_tag   
        return True

    def work(self):
        rospy.wait_for_service('/move_cube')
        move_cube = rospy.ServiceProxy('/move_cube', MoveCubeRequest)
        self.move_cube = move_cube
        print("move cube available!")
        song_request_service = rospy.Service('/song_request_channel', NewSongRequest, self.handle_song_request)
        change_song_service = rospy.Service('/change_song_service', SongEnding, self.handle_change_song)
        rospy.spin()



if __name__ == "__main__":
    try:
        rospy.init_node('decision_maker')
        d = DecisionMaker([11, 17], 16)
        d.work()
    except rospy.ROSInterruptException as e:
        print(e)
        pass