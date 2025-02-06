#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
import math


# Constants
FORWARD = 1
BACKWARD = -1
N_TOTAL_TICKS = 135
PI = math.pi
DISTANCE = 1.25




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
        
        # temporary data storage
        self._ticks_left = rospy.wait_for_message(self._left_encoder_topic, WheelEncoderStamped).data
        self._ticks_right = rospy.wait_for_message(self._left_encoder_topic, WheelEncoderStamped).data

        self._wheel_radius = rospy.get_param(f"/{vehicle_name}/kinematics_node/radius")
        print(self._wheel_radius)

        self._total_distance_in_ticks = (DISTANCE * N_TOTAL_TICKS) / (2 * PI * self._wheel_radius)
        print(self._total_distance_in_ticks)
        self._direction = FORWARD
        
        # form the initial velocity to publish
        self._vel_left = 0.4
        self._vel_right = 0.4
        
        

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(0.1)

        # Get initial ticks
        initial_left_ticks = self._ticks_left
        initial_right_ticks = self._ticks_right

        while not rospy.is_shutdown():
            # get distance travelled so far in ticks

            # print(self._ticks_left)
            # print(self._ticks_left)
            
            distance_travelled_left = abs(self._ticks_left - initial_left_ticks)
            distance_travelled_right = abs(self._ticks_right - initial_right_ticks)
            distance_travelled = (distance_travelled_left + distance_travelled_right) / 2
            rospy.loginfo(f"distance travelled: {distance_travelled}")

            if (distance_travelled >= self._total_distance_in_ticks):
                print("max distance travelled")
                if (self._direction == FORWARD):
                    print("got here")
                    self._direction = BACKWARD
                    self._vel_left = self._vel_left * self._direction
                    self._vel_right = self._vel_right * self._direction
                    initial_left_ticks = self._ticks_left
                    initial_right_ticks = self._ticks_right
                else:
                    print("got to on shutdown")
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