#!/usr/bin/env python
'''
This script runs a rospy service that adds songs to the queue.
The service receives either: 1) a request with a specific AR tag
or 2) a request with no specific AR tag. In the case of 2), the
service will simply pop the next item from the queue.
'''
import rospy
import requests
import base64
from std_msgs.msg import Int32
from std_msgs.msg import String

NOW_PLAYING_ENDPOINT = 'https://api.spotify.com/v1/me/player/currently-playing';
QUEUE_ENDPOINT = 'https://api.spotify.com/v1/me/player/queue';
TOKEN_ENDPOINT = 'https://accounts.spotify.com/api/token';

client_id='d0a17e713ff0462eb90dc5f7cff7083c'
client_secret='b5260a40d8fb4ecb9f16edcfb6e95546'
refresh_token="AQCWOZWOjBd2U0bA_XCYAyngCPI2dQ6sgj98omAC2HYFJiB9oKYoRClj_1L9TSnvWmGH1lnXryBr9ChxCmhCCuunD0B5YvO3-QEumy3KrzpomQs0X8DiVqKR-wYDJWntRbk"

basic = "ZDBhMTdlNzEzZmYwNDYyZWI5MGRjNWY3Y2ZmNzA4M2M6YjUyNjBhNDBkOGZiNGVjYjlmMTZlZGNmYjZlOTU1NDY="
token = None

class AddSongService:
    #Callback for when song is added to Spotify queue
    def handle_add_song(self, msg):
        #TODO: fix this
        if msg.ar_tag == -1:
            if len(queue) > 0:
                ar_tag, song = queue.pop(0)
                add_song_to_queue(song) # should we publish the ar tag we chose so it can be available?
        else:
            for i in range(len(queue)):
                if queue[i][0] = msg.ar_tag:
                    ar_tag, song = queue.pop(0)
                    add_song_to_queue(song)
                    return


    #Callback for when song is added to local queue (aka human adds ar tag: song pair)
    def handle_add_song(self, msg):
        #TODO: fix this
        self.queue.append([msg.ar_tag, msg.song])

    def __init__self():
        self.queue = []
        rospy.init_node("song_adder")
        rospy.Subscriber('/song_requests', String, self.handle_song_request)
        rospy.Service("add_song_to_queue", AddSong, self.handle_add_song)


def refresh_the_token(): 
    global token
    auth_client = client_id + ":" + client_secret
    auth_encode = 'Basic ' + base64.b64encode(auth_client.encode()).decode()

    headers = {
        'Authorization': auth_encode,
        }

    data = {
        'grant_type' : 'refresh_token',
        'refresh_token' : refresh_token
        }

    response = requests.post('https://accounts.spotify.com/api/token', data=data, headers=headers) #sends request off to spotify

    if(response.status_code == 200): #checks if request was valid
        print("The request to went through we got a status 200; Spotify token refreshed")
        response_json = response.json()
        new_expire = response_json['expires_in']
        token = response_json["access_token"]
        return response_json["access_token"]
    else:
        print("ERROR! The response we got was: "+ str(response))

# def get_currently_playing():
#     global token
#     if token == None:
#         refresh_the_token()
#     headers = {
#         'Authorization': "Bearer " + token
#     }
#     response = requests.get(NOW_PLAYING_ENDPOINT, headers=headers)
#     response_json = response.json()
#     progress_ms, duration_ms, url = response_json["progress_ms"], response_json["item"]["duration_ms"], response_json["item"]["external_urls"]["spotify"]
#     return duration_ms - progress_ms
#     #    switch_song(url)

# def switch_song(url):
#     global current_song
#     if url != current_song:
#         # do nothing, we have not supplied a new song already
#         current_song = url
#         if len(queue) > 0:
#             new_song = queue.pop(0)
#             # add song to Spotify queue
#             next_uri = new_song[1]
#             next_ar_tag = new_song[0]
#             # send signal that need to choose new ar tag
def add_song_to_queue(song):
    global token
    if token == None:
        refresh_the_token()
    uri = ""
    if "://" in song:
        uri = song.split("/")
        uri = uri[-1]
        if "?" in uri:
            uri = uri.split("?")[0]
    else:
        uri = song
        
    api_url = "https://api.spotify.com/v1/me/player/queue?uri=spotify%3Atrack%3A" + uri
    headers = {
        'Authorization': "Bearer " + token,
        'Content-Type': 'application/json'
    }

    response = requests.post(api_url, headers=headers)

if __name__ == "__main__":
    node = AddSongService()# look at lab 4
    node.run()

