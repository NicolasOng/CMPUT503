#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun computer_vision color_based_movement.py

# wait for app to end
dt-launchfile-join
