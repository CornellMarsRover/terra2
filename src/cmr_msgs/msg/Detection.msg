# Single object detection for the URC 2026 autonomy mission.
#
# Produced by the base-station detection node (cmr_cams base_detection) from the
# live ZED feed and consumed by:
#   * the telemetry GUI  -> draws exactly ONE bounding box + "OBJECT DETECTED" banner
#   * the rover state machine -> arrival / stop sequencing
#
# At most ONE detection is ever reported (the highest-confidence valid one), per
# the URC rule that only one object may be highlighted at a time.

std_msgs/Header header

string label          # detected class: "mallet" | "bottle" | "ice_pick"; "" if none
int32 class_id        # YOLO class id, or -1 if no detection
float32 confidence    # detection confidence in [0, 1]

# Bounding box, normalized to the source image in [0, 1] so the overlay is
# resolution-independent. Only meaningful when label is non-empty.
float32 x1
float32 y1
float32 x2
float32 y2

# True when the chosen detection matches the currently-armed target above the
# stop confidence threshold -> the rover's onboard arrived/detected decision.
bool detected
