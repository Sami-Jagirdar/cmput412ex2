#!/usr/bin/env python3
import rosbag
import os
import math

theta_init = math.pi/2

def get_x_i(x_r, theta):
    return x_r * math.cos(theta)

def get_y_i(x_r, theta):
    return x_r * math.sin(theta)


vehicle_name = 'csc22926'
wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"

bag = rosbag.Bag('D3.bag')
count=0
for topic, msg, t in bag.read_messages(topics=[wheels_topic]):
    print(msg)
    count+=1

print(count)
bag.close()