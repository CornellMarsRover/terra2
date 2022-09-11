#!/bin/bash

ros2 launch cmr_demo demo.launch.py
active_nodes=$(ros2 node list)
if ! [[ $active_nodes == *"/demo1"* ]]; then
    echo "Test failed: demo1 node not active"
    exit 1
fi
if ! [[ $active_nodes == *"/demo2"* ]]; then
    echo "Test failed: demo2 node not active"
    exit 1
fi

# ros2 topic 

