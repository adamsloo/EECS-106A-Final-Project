# import rospy
# from your_package_name.srv import ProcessQueue, ProcessQueueResponse
# from std_msgs.msg import String

# class QueueProcessorNode:
#     def __init__(self):
#         rospy.init_node('queue_processor_node')

#         # Create a service to process the queue
#         self.service = rospy.Service('process_queue', ProcessQueue, self.process_queue)

#         # Queue to store messages
#         self.message_queue = []

#     def process_queue(self, request):
#         rospy.loginfo("Received queue for processing: %s", request.queue)

#         # Process the queue (for demonstration purposes, just convert strings to uppercase)
#         processed_queue = [msg.upper() for msg in request.queue]

#         rospy.loginfo("Processed queue: %s", processed_queue)

#         # Return the remaining queue
#         remaining_queue = [msg for msg in self.message_queue if msg not in processed_queue]
#         self.message_queue = remaining_queue

#         return ProcessQueueResponse(remaining_queue=remaining_queue)

#     def spin(self):
#         rospy.spin()

# if __name__ == '__main__':
#     queue_processor = QueueProcessorNode()
#     queue_processor.spin()


# # class QueueProcessorNode:
# #     def __init__(self):
# #         rospy.init_node('queue_processor_node', anonymous=True)
        
# #         self.queue = []
# #         self.processing_time = rospy.get_param('~processing_time', 1.0)  # Default processing time is 1 second
# #         self.loop_rate = rospy.Rate(10)  # 10 Hz

# #         rospy.Subscriber('/add_to_queue', NewSongRequest, self.add_to_queue_callback)
# #         self.subscriber = rospy.Subscriber('/song_changing', bool, self.song_changing_callback)
    
# #     def song_changing_callback(self, msg):
# #         # Callback function that stores the received message
# #         self.received_message = msg.data

# #     def add_to_queue_callback(self, data):
# #         rospy.loginfo('Adding to queue: %s', data.ar_tag, data.song)
# #         self.queue.append([data.ar_tag, data.song])

# #     def process_queue(self):
# #         while not rospy.is_shutdown():
# #             if len(self.queue) > 0:
# #                 ar_tag, song = self.queue.pop(0)
# #                 add_song_to_queue(song) 
# #                 # self.taken_tags.remove(ar_tag)
# #                 # self.available_tags.append(ar_tag)
# #                 print("MOVING CUBE")
# #                 self.move_cube(int(ar_tag), int(self.prev_ar_tag))
# #                 self.prev_ar_tag = ar_tag

# #                 #time how long move cube takes
# #                 time.sleep(self.processing_time)

# #             # Check if there is time left, otherwise wait
# #             self.loop_rate.sleep()

# # if __name__ == '__main__':
# #     try:
# #         node = QueueProcessorNode()
# #         node.process_queue()
# #     except rospy.ROSInterruptException:
# #         pass
