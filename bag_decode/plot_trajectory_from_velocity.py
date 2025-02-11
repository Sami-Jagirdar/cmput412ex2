#!/usr/bin/env python3
import rosbag
import os
import math
import numpy as np
import matplotlib.pyplot as plt

WHEEL_RADIUS = 0.0318
WHEEL_BASE = 0.09
SCALING_FACTOR = 7.5 # The velocities received from the wheel_driver_node are not exact velocities
# Therefore, the change in angles which depend on the actual velocity values are affected
# The throttle values received are propotional to the actual velocities, but the propotionality constants
# are not known, so I tried using different values manually until I would get a reasonable plot

vehicle_name = 'csc22926'
wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"

# Change to the bag you want to use
bag = rosbag.Bag('D2.bag')

previous_time = None
xi = 0
yi = 0
theta = math.pi/2 # starts vertically 
time_elapsed = 0
trajectory = [(xi,yi)] # Start at origin in world frame

for topic, msg, t in bag.read_messages(topics=[wheels_topic]):
    
    current_time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9

    if previous_time is not None:
        dt = current_time - previous_time # Fist timestamp is a huge value, so we start doing the time changes from the second timestamp
    else:
        dt = 0.05 # Default initial time stamp

    previous_time = current_time # for next iteration

    # Associating the throttle to the wheel from the message to the angular velocity of the wheel
    v_left = msg.vel_left * WHEEL_RADIUS
    v_right = msg.vel_right * WHEEL_RADIUS

    # print(f"v_left: {v_left}")
    # print(f"v_right: {v_right}")
    # print(f"dt: {dt}")

    # Linear and Angular velocity of the robot
    change_in_xr = ((v_right + v_left) / 2) * SCALING_FACTOR # From kinematics equation
    
    change_in_theta_r = ((v_right - v_left) / WHEEL_BASE) * SCALING_FACTOR # From kinematics equation
    # change_in_yr = 0 always

    xi += change_in_xr * math.cos(theta) * dt
    yi += change_in_xr * math.sin(theta) * dt
    theta += change_in_theta_r * dt

    trajectory.append((xi,yi))

bag.close()

trajectory = np.array(trajectory)
plt.figure()
plt.plot(trajectory[:,0], trajectory[:,1], label="Duckiebot Trajectory")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.axis("equal")
plt.show()

