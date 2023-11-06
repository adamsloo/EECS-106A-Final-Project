# add a server
import os
from flask import Flask, request, render_template, jsonify
from std_msgs.msg import String
import rospy

app = Flask(__name__)

available_ars = [15, 17, 13]
ar_to_song_mappings = {}
queue = []

@app.route('/request', methods=['POST'])
def request_song():
	try:
		data = request.get_json()
		ar_tag = data["ar_tag"]
		song = data["song"]
		if int(ar_tag) in available_ars:
			available_ars.remove(int(ar_tag))
			ar_to_song_mappings[ar_tag] = song
	except Exception as e:
		return (e, 400)
	print(song)
	return "ok"

def add_song_to_queue(msg):
	queue.append(msg.data)
	print(queue)

if __name__ == '__main__':
	rospy.init_node("song_queue")
	rospy.Subscriber("song_queue", String, add_song_to_queue)
	app.run(debug=True)
	rospy.spin()
