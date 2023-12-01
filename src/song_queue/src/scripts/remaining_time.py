#!/usr/bin/env python
import rospy
import requests
import base64
from std_msgs.msg import Int32
from std_msgs.msg import String
from song_queue.srv import SongEnding

NOW_PLAYING_ENDPOINT = 'https://api.spotify.com/v1/me/player/currently-playing';
QUEUE_ENDPOINT = 'https://api.spotify.com/v1/me/player/queue';
TOKEN_ENDPOINT = 'https://accounts.spotify.com/api/token';

client_id='d0a17e713ff0462eb90dc5f7cff7083c'
client_secret='b5260a40d8fb4ecb9f16edcfb6e95546'
refresh_token="AQCWOZWOjBd2U0bA_XCYAyngCPI2dQ6sgj98omAC2HYFJiB9oKYoRClj_1L9TSnvWmGH1lnXryBr9ChxCmhCCuunD0B5YvO3-QEumy3KrzpomQs0X8DiVqKR-wYDJWntRbk"

basic = "ZDBhMTdlNzEzZmYwNDYyZWI5MGRjNWY3Y2ZmNzA4M2M6YjUyNjBhNDBkOGZiNGVjYjlmMTZlZGNmYjZlOTU1NDY="
token = None

current_song = ""

def main():
    rospy.wait_for_service('change_song')
    change_song = rospy.ServiceProxy('change_song', SongEnding)

    while not rospy.is_shutdown():
        time_left, uri = get_currently_playing()
        if current_song != uri and time_left < 30000: # we haven't sent the message yet
            current_song = uri
            resp = change_song(time_left)
            

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

def get_currently_playing():
    global token
    if token == None:
        refresh_the_token()
    headers = {
        'Authorization': "Bearer " + token,
        'Content-Type':'application/json'
    }
    print("poop")
    response = requests.get(NOW_PLAYING_ENDPOINT, headers=headers)
    print(response)
    response_json = response.json()
    progress_ms, duration_ms, url = response_json["progress_ms"], response_json["item"]["duration_ms"], response_json["item"]["external_urls"]["spotify"]
    uri = response_json["item"["uri"]]
    return duration_ms - progress_ms, uri

if __name__ == "__main__":
    get_currently_playing()
    #rospy.init_node('remaining_time', anonymous=True)
    #main()

