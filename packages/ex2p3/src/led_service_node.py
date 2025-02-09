#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Int32
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA

import os

# States
STATE_1 = 1
STATE_2 = 2
STATE_3 = 3


class LEDServiceNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LEDServiceNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        vehicle_name = os.environ['VEHICLE_NAME']
        led_topic = f"/{vehicle_name}/led_state"
        led_emitter_topic = f"/{vehicle_name}/led_emitter_node/led_pattern"

        # construct subscriber
        self.sub = rospy.Subscriber(led_topic, Int32, self.callback)
        self.pub = rospy.Publisher(led_emitter_topic, LEDPattern, queue_size=10)
    
    def getLEDColor(self, red, green, blue):
        led_colors = LEDPattern()

        # 5 LEDs on the duckiebot
        for i in range(5):
            rgba = ColorRGBA()
            rgba.r = red
            rgba.g = green
            rgba.b = blue
            rgba.a = 1.0
            led_colors.rgb_vals.append(rgba)

        return led_colors
    
    def callback(self, data):
        rospy.loginfo("State received: '%d'", data.data)

        if (data.data == STATE_1):
            self.pub.publish(self.getLEDColor(1,0,0))
        elif (data.data == STATE_2):
            self.pub.publish(self.getLEDColor(0,1,0))
        elif (data.data == STATE_3):
            self.pub.publish(self.getLEDColor(0,0,1))
        else:
            rospy.loginfo("Invalid State received: '%d'", data.data)



if __name__ == '__main__':
    # create the node
    node = LEDServiceNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()
