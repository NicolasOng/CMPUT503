#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun computer_vision camera_detection.py

# wait for app to end
dt-launchfile-join
