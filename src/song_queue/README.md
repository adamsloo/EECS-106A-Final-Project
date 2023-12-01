add_song_to_queue --> takes in song url adds song to queue
remaining_time --> publishes remaining time in current song
request_taker --> takes in user input, pings decision_maker service
decision_maker --> stores visible ar tags, available ar tags, taken ar tags, makes decision based on robot

THE WORKFLOW
request_taker:
- reads in available_tags from decision_maker
- takes input from user, sends [ar_tag, song] to decision_maker
remaining_time:
- checks when there are 30 seconds left, sends service request to decision_maker
decision_maker:
- reads in visible ar tags from robot (subscriber)
- when request_taker sends service request: (service song_request_channel)
    - adds [ar_tag, song] to queue
    - removes ar_tag from available_tags
    - adds ar_tag to taken_tags
- when remaining_time sends service request for 30 seconds: (service change_song)
    - chooses a new song from queue
    - sends service request to robot saying what ar tag to get next (send [prev_ar_tag, next_ar_tag])
- when remaining_time sends service request for 5 seconds:
    - remove current ar_tag from taken_tags
    - set current_ar_tag to new ar_tag