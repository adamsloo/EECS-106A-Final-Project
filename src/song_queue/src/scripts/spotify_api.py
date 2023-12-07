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

def pause_song():
    global token
    if token == None:
        refresh_the_token()
    api_url = "https://api.spotify.com/v1/me/player/pause"
    headers = {
        'Authorization': "Bearer " + token,
    }
    response = requests.post(api_url, headers=headers)

def play_song():
    global token
    if token == None:
        refresh_the_token()
    api_url = "https://api.spotify.com/v1/me/player/play"
    headers = {
        'Authorization': "Bearer " + token,
    }
    response = requests.post(api_url, headers=headers)


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

# if __name__ == "__main__":
#     rospy.init_node("spotify_api")
#     rospy.Service("add_song_to_queue", String, add_song_to_queue)
