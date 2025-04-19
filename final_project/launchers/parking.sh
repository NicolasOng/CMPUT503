#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun safety_detection parking.py

# wait for app to end
dt-launchfile-join
