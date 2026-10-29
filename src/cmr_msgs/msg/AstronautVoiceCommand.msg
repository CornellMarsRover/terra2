std_msgs/Header header

# One of FOLLOW, STAY, FETCH, COME, or GIVE.
string command
float32 confidence
string transcript

# Must be VOICE. The fusion node rejects blank or unknown sources.
string source
