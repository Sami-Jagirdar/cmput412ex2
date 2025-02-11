#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
import math
from std_msgs.msg import Int32


# Constants
RESOLUTION = 135 # DB Series duckiebot resolution is 135
PI = math.pi

# States
STATE_1 = 1
STATE_2 = 2
STATE_3 = 3


class WheelOfDNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelOfDNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        led_topic = f"/{vehicle_name}/led_state"
        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"
        
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self._led_publisher = rospy.Publisher(led_topic, Int32, queue_size=1)

        # construct subscriber
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        
        # Get the initial ticks rotated at the time of starting the program
        self._ticks_left = rospy.wait_for_message(self._left_encoder_topic, WheelEncoderStamped).data
        self._ticks_right = rospy.wait_for_message(self._right_encoder_topic, WheelEncoderStamped).data

        # At the time of running my code, I get 0.0318m as the radius from ros
        # This is very close to the typical radius of the DB series bot of 0.0325m
        self._wheel_radius = rospy.get_param(f"/{vehicle_name}/kinematics_node/radius")

        self._wheelbase = 0.09 # Manually measured
        
        # form the initial velocity to publish
        self._vel_left = 0
        self._vel_right = 0

        self._rate = rospy.Rate(20)

    def run(self):

        while not rospy.is_shutdown():

            self.state_one()
            self.state_two()
            self.state_three()
            self.state_one()
            # self.curve_right(PI/2, 0.20)
            # self.move_straight(1.2)
            # self.rotate_90()
            break

    def stop(self):
        self.publish_velocity(0,0)

    def on_shutdown(self):
        self.stop()
        rospy.signal_shutdown("manually shutdown")

    def callback_left(self, data):
        self._ticks_left = data.data

    def callback_right(self, data):
        self._ticks_right = data.data

    def publish_velocity(self, vel_left, vel_right):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = vel_left
        msg.vel_right = vel_right
        self._publisher.publish(msg)

    def state_one(self):
        self._led_publisher.publish(STATE_1)
        self.stop()
        rospy.sleep(5)

    def state_two(self):
        self._led_publisher.publish(STATE_2)

        # Travel 1.2 m straight
        self.move_straight(1.2, first=True)
        # Rotate 90 degree clockwise
        self.rotate_90()
        # Travel 0.92 m straight
        self.move_straight(0.80)
        # Curve right
        self.curve_right(PI/2, 0.20)
        # Travel 0.35 m straight
        self.move_straight(0.35)
        # Curve right
        self.curve_right(PI/2, 0.20)
        # Travel 0.92 m straight
        self.move_straight(0.80)

    def state_three(self):
        self._led_publisher.publish(STATE_3)
        self.rotate_90()
        self.stop()
        rospy.sleep(5)

    
    def move_straight(self, length, first=False):
        print("straight line \n")
        initial_left_ticks = self._ticks_left
        initial_right_ticks = self._ticks_right

        distance_travelled = 0
        straight_line_a = (length * RESOLUTION) / (2 * PI * self._wheel_radius)
        print("Straight line a: " + str(straight_line_a))


        self._vel_left = 0.35
        if (first):
            self._vel_right = 0.38
        else:
            self._vel_right = 0.35

        while (distance_travelled < straight_line_a):
            self.publish_velocity(self._vel_left, self._vel_right)

            distance_travelled_left = abs(self._ticks_left - initial_left_ticks)
            distance_travelled_right = abs(self._ticks_right - initial_right_ticks)
            distance_travelled = (distance_travelled_left + distance_travelled_right) / 2
            print("distance travelled: "+str(distance_travelled))
            self._rate.sleep()

        print("straight line done\n")
        self.stop()

    def rotate_90(self):
        print("rotating 90\n")
        initial_left_ticks = self._ticks_left
        initial_right_ticks = self._ticks_right

        self._vel_left = 0.38
        self._vel_right = -0.38

        angle = 0

        while (angle <= (PI-0.33)/2):
            self.publish_velocity(self._vel_left, self._vel_right)

            distance_travelled_left = abs(self._ticks_left - initial_left_ticks) * (2 * PI * self._wheel_radius) / RESOLUTION
            distance_travelled_right = abs(self._ticks_right - initial_right_ticks) * (2 * PI * self._wheel_radius) / RESOLUTION
            angle = (distance_travelled_left + distance_travelled_right) / self._wheelbase
            print("angle: "+str(angle))

            self._rate.sleep()

        print("rotated 90\n")
        self.stop()

    def curve_right(self, angle_to_travel, radius):
        print("Curving right\n")
        initial_left_ticks = self._ticks_left
        initial_right_ticks = self._ticks_right

        arc_radius_right_wheel = radius - self._wheelbase / 2
        arc_radius_left_wheel = radius + self._wheelbase / 2
        distance_right_wheel = arc_radius_right_wheel * angle_to_travel
        distance_left_wheel = arc_radius_left_wheel * angle_to_travel

        self._vel_right = 0.35
        self._vel_left = (distance_left_wheel / distance_right_wheel) * self._vel_right

        angle_travelled = 0

        while (angle_travelled <= angle_to_travel):
            self.publish_velocity(self._vel_left, self._vel_right)

            distance_travelled_left = abs(self._ticks_left - initial_left_ticks) * (2 * PI * self._wheel_radius) / RESOLUTION
            distance_travelled_right = abs(self._ticks_right - initial_right_ticks) * (2 * PI * self._wheel_radius) / RESOLUTION
            angle_travelled = abs((distance_travelled_right - distance_travelled_left) / self._wheelbase)
            print("angle travelled: " + str(angle_travelled))

            self._rate.sleep()

        print("curve done\n")

        self.stop()







if __name__ == '__main__':
    # create the node
    node = WheelOfDNode(node_name='wheel_control_node')
    # run node
    node.run()
    
    # terminate the process
    rospy.signal_shutdown("Completed straight line task")
