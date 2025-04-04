#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun safety_detection odometry.py

# wait for app to end
dt-launchfile-join
