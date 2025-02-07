#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
import math

RESOLUTION = 135
CLOCKWISE = 1
COUNTER_CLOCKWISE = -1
PI = math.pi

class WheelControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # construct subscriber
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        
        # Get the initial ticks rotated at the time of starting the program
        self._ticks_left = rospy.wait_for_message(self._left_encoder_topic, WheelEncoderStamped).data
        self._ticks_right = rospy.wait_for_message(self._right_encoder_topic, WheelEncoderStamped).data

        # At the time of running my code, I get 0.0318m as the radius from ros
        # This is very close to the typical radius of the DB series bot of 0.0325m
        self._wheel_radius = rospy.get_param(f"/{vehicle_name}/kinematics_node/radius")
        print(self._wheel_radius)
        self._wheelbase = 0.09 # Manually measured this on the duckiebot and callibrated
        self._direction = CLOCKWISE
        
        # form the initial velocity to publish
        self._vel_left = 0.3
        self._vel_right = -0.3

    def run(self):
        rate = rospy.Rate(20)

        # Get initial ticks
        initial_left_ticks = self._ticks_left
        initial_right_ticks = self._ticks_right

        self._vel_left = 0.3
        self._vel_right = -0.3

        while not rospy.is_shutdown():

            # get distance travelled (arc length) by left wheel            
            distance_travelled_left = abs(self._ticks_left - initial_left_ticks) * (2 * PI * self._wheel_radius) / RESOLUTION
            # get distance travelled (arc length) by right wheel
            distance_travelled_right = abs(self._ticks_right - initial_right_ticks) * (2 * PI * self._wheel_radius) / RESOLUTION
            # get final angle travelled about the center of the wheel base
            angle = (distance_travelled_left + distance_travelled_right) / self._wheelbase
            rospy.loginfo(f"angle rotated: {angle}")


            if (angle >= PI/2):

                if (self._direction == CLOCKWISE):
                    print("Rotated 90 degrees clockwise")

                    # Change direction of rotation and reset ticks
                    self._direction = COUNTER_CLOCKWISE
                    self._vel_left = self._vel_left * self._direction
                    self._vel_right = self._vel_right * self._direction
                    initial_left_ticks = self._ticks_left
                    initial_right_ticks = self._ticks_right

                else:
                    print("Rotated 90 degrees counter clockwise")
                    self.on_shutdown()
                    break

            message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
            self._publisher.publish(message)
            print("published")
            
            rate.sleep()

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)


    def callback_left(self, data):
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.data}")
        self._ticks_left = data.data

    def callback_right(self, data):
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.data}")
        self._ticks_right = data.data


if __name__ == '__main__':
    # create the node
    node = WheelControlNode(node_name='wheel_control_node')
    # run node
    node.run()
    # keep the process from terminating
    # rospy.spin()