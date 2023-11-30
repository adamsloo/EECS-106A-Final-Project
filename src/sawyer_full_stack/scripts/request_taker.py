#!/usr/bin/env python

import rospy
# from sawyer_full_stack.msg import SongRequest
from std_msgs.msg import String
import rospkg



available_tags = [13, 14, 15]
taken_tags = []
queue = []

def talker():
    pub = rospy.Publisher('/song_requests', String, queue_size=10)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        # see available ar tags
        request = input("Please enter an ar tag and song url/uri pair. The available ar tags are " + str(available_tags))
        ar_tag, song = request.split(" ")
        if int(ar_tag) not in available_tags:
            print("Unavailable AR tag. Please try again.")
        else:
            queue.append([ar_tag, song])
            available_tags.remove(int(ar_tag))
            taken_tags.append(int(ar_tag))
            song_request = String(ar_tag + "," + song)
            pub.publish(song_request)
            print("Current queue:" + queue)
            r.sleep()

if __name__ == "__main__":
    rospy.init_node('talker')

    try:
        talker()
    except rospy.ROSInterruptException: pass