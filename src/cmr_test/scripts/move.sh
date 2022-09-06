#!/bin/bash

ros2 topic pub --once /test/set_target geometry_msgs/Point  "{ x: $1, y: $2, z: 0 }"