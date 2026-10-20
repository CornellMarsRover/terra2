std_msgs/Header header

# Stable pose classification and the command that is safe for consumers to use.
# FOLLOW/FETCH are withheld as NONE when distance_valid is false.
string gesture
string command
float32 confidence

bool astronaut_detected
bool distance_required
bool distance_valid
string depth_status
float32 distance_m

# ZED camera-frame target: x=forward, y=left, z=up, in meters.
geometry_msgs/Point position
string direction
float32 horizontal_angle_deg
uint32 depth_samples
