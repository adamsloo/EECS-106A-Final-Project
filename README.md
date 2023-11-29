# EECS-106A-Final-Project
Isabella Borkovic, Heather Ding, Adam Loo, Justin Zhang

## Project Structure

### ar_track_alvar
* 

### baxter_pykdl
* 

### dance_moves
* originally copy of lab 5 files
* changed ik_examples.py to move the arm to the three different dance positions
* changed pick_place.py to repeatedly dance according to the three dance positions

### find_markers
* find_markers.py
    * Examens all the markers in the workspace (need to tuck so that it can see workspace previous)
    * Publishes list of markers to topic "/seen_markers"

### sawyer_full_stack
* 

### song_queue
* pick_up.py
    * 
* server.py
    * flask app that lets users interact with a website and add songs to the queue 
* upload_photo.py
    * When this "song_tracker" node receives a new "Marker" message, it adds it to the "current_ids" dictionary. It publishes the song id string message to "song_queue". 
    * This node add songs we want to play to the queue
