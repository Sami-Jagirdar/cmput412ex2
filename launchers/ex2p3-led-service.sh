#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun ex2p3 led_service_node.py

# wait for app to end
dt-launchfile-join