#!/usr/bin/env python3
import rosbag
import os

vehicle_name = os.environ['VEHICLE_NAME']
wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"

bag = rosbag.Bag('move.bag')
for topic, msg, t in bag.read_messages(topics=[wheels_topic]):
    print(msg)
bag.close()