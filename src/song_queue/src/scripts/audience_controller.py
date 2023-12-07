#!/usr/bin/env python

import random 
import rospy
import rospkg
import roslaunch
from collections import deque
from ar_track_alvar_msgs.msg import AlvarMarkers

import rospy
from std_msgs.msg import Int64
# Node that listens in to head ar tracker and commands script to pick and place correct node 
class AudienceControllerNode:
    def __init__(self):
        self.queue = deque(maxlen=20) #queue storing deltas of marker position
        self.x_delta_sum = 0
        self.y_delta_sum = 0
        self.queue_max = 20
        self.prev_position = None

        self.x_bottom_thresh = 0.5
        self.y_bottom_threshold = 0.5
        self.x_top_thresh = 2
        self.y_bottom_threshold = 2

        self.huge_number = 200

        rospy.init_node('audience_controller')
        # Subscribe to the topic that publishes ar pose data
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_marker_callback)

    def ar_marker_callback(self, data):
        if len(data.markers) > 0:
            if self.prev_position is None:
                marker = data.markers[0]
                self.prev_position = marker.pose.pose.position
            else:
                marker = data.markers[0]
                cur_val = marker.pose.pose.position
                self.queue_x.append(cur_val.x)  
                self.queue_y.append(cur_val.y)  
        # else we dont see ar tag
        else:
            self.queue_x.append(10000000)  
            self.queue_y.append(10000000)  
        
        # Clean up queue
        if len(self.queue) == self.queue_max:
            self.queue.popleft()
            print(self.check_condition(self.x_queue, self.y_queue))

    def ar_marker_callback2(self, data):
        # Callback function to process incoming data from the topic
        # self.seen_markers = set([int(marker.id) for marker in data.markers] ) # set of currently seen markers
        # self.handle_queue_audience_song()
        if len(data.markers) > 0:
            if self.prev_position is None:
                marker = data.markers[0]
                self.prev_position = marker.pose.pose.position
            else:
                marker = data.markers[0]
                cur_val = marker.pose.pose.position
                x_delta = abs(cur_val.x - self.prev_position.x)
                y_delta = abs(cur_val.y - self.prev_position.y)
                self.x_delta_sum += x_delta
                self.y_delta_sum += y_delta
                self.queue.append((x_delta, y_delta))  
        # else we dont see ar tag
        else:
            x_delta = self.huge_number      # this way no condition will be triggered until
            y_delta = self.huge_number      # this set is out of queue
            self.x_delta_sum += x_delta
            self.y_delta_sum += y_delta
            self.queue.append((x_delta, y_delta))  
        
        # Clean up queue
        if len(self.queue) == self.queue_max:
            oldest_val = self.queue.popleft()
            self.x_delta_sum -= oldest_val[0]
            self.y_delta_sum -= oldest_val[1]
            print("x", self.x_delta_sum)
            print("y", self.y_delta_sum)
            print(self.check_condition(self.x_delta_sum, self.y_delta_sum))

    def check_condition(self, x_d, y_d):
        x_condition = max(x_d) - min(x_d) > 50 and max(x_d) - min(x_d) < 900
        y_condition = max(y_d) - min(y_d) > 50 and max(y_d) - min(y_d) < 900
        print(max(x_d) - min(x_d))
        if x_condition and not y_condition:
            return "x"
        if y_condition and not x_condition:
            return "y"

    def check_condition2(self, x_d, y_d):
        if x_d > self.x_top_thresh and y_d < self.y_bottom_threshold:
            return "PLAY"
        elif x_d < self.x_bottom_thresh and y_d > self.y_top_threshold:
            return "PAUSE"
        
    
        

if __name__ == '__main__':
    try:
        audience_controller = AudienceControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
