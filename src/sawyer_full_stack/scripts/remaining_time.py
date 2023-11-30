#!/usr/bin/env python
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

current_song = ""
next_song = ""

visible_ar_tags = []
available_ar_tags = [15, 17]
queue = []

def main():
    pub = rospy.Publisher("/time_left_ms", Int32, queue_size=10) #publishes how many milliseconds are left

    while not rospy.is_shutdown():
        time_left = Int32(get_currently_playing())
        pub.publish(time_left)



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
        'Authorization': "Bearer " + token
    }
    response = requests.get(NOW_PLAYING_ENDPOINT, headers=headers)
    response_json = response.json()
    progress_ms, duration_ms, url = response_json["progress_ms"], response_json["item"]["duration_ms"], response_json["item"]["external_urls"]["spotify"]
    return duration_ms - progress_ms
    #    switch_song(url)

if __name__ == "__main__":
    rospy.init_node('remaining_time', anonymous=True)
    main()

