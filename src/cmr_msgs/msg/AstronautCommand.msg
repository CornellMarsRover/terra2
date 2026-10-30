std_msgs/Header header

# Stable pose classification and the command that is safe for consumers to use.
# FOLLOW is withheld as NONE when distance_valid is false. FETCH recognition is
# independent of astronaut depth; the arm must localize the nearby object.
string gesture
string command
float32 confidence

# Arbitration metadata. safety_stop is true only for a generated hold (for
# example, conflicting modalities), not an astronaut's explicit STAY command.
string source
string decision_reason
bool safety_stop

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
