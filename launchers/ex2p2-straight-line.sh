#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun ex2p2 straight_line_task.py

# wait for app to end
dt-launchfile-join