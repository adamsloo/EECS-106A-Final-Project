#!/usr/bin/env python

import rospy
from song_queue.srv import SongRequest #TODO: THIS DOESNT WORK
from std_msgs.msg import String
from song_queue.msg import AvailableArTags
import rospkg

available_tags = [15, 17]

def update_available_tags(msg):
    global available_tags
    available_tags = msg.ar_tags

def main():
    rospy.wait_for_service('song_requests_channel')
    song_requests_channel = rospy.ServiceProxy('song_requests_channel', SongRequest)

    while not rospy.is_shutdown():
        # see available ar tags
        request = input("Please enter an ar tag and song url/uri pair. The available ar tags are " + str(available_tags))
        ar_tag, song = request.split(" ")
        if int(ar_tag) not in available_tags:
            print("Unavailable AR tag. Please try again.")
        else:
            respl = song_requests_channel(ar_tag, song)

if __name__ == "__main__":
    rospy.init_node('talker')
    try:
        rospy.Subscriber("available_tags", AvailableArTags, update_available_tags)
        rospy.spin()

        main()
    except rospy.ROSInterruptException: pass