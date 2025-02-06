#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun ex2p1 my_subscriber_node.py

# wait for app to end
dt-launchfile-join
