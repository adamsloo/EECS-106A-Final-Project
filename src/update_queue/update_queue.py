from socket import send_fds
import rospy
from std_msgs.msg import String  # Import the message type you want to subscribe to
import random

class UpdateQueue:
    def __init__(self):
        rospy.init_node('update_queue', anonymous=True)
        self.tag_to_update = None

        self.pub = rospy.Publisher('/curr_tag_processing', String, queue_size=10)
        self.sub = rospy.Subscriber('/seen_markers', String, self.choose_next_ar_tag)
        rospy.spin()

    # If current song is over, choose random AR tag to move
    def choose_next_ar_tag(self, data):
        rospy.loginfo("Received data: %s", data.data)
        ar_tags = data.data

        # TODO: GET INFO FROM BACKEND ON WHEN A SON IS CLOSE TO OVER
        if len(ar_tags) == 0:
            print("NO AR TAGS IN WORKSPACE")
        else:
            self.tag_to_update = random.choice(ar_tags)
            self.send_to_frontend(self.tag_to_update)
            self.pub.publish(self.tag_to_update)

    # Send AR tag to front end to process music playing
    def send_to_frontend(self, tag_to_update):
        ...

if __name__ == '__main__':
    try:
        UpdateQueue()
    except rospy.ROSInterruptException:
        pass