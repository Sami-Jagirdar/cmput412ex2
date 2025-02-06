#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import math

import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._custom_topic = f"/{self._vehicle_name}/camera_node/annotated_image/compressed"
        # bridge between OpenCV and ROS

        self._bridge = CvBridge()
        # create window
        self._window = "ex2p1-camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        # construct publisher to publish annotated image to custom topic
        self.pub = rospy.Publisher(self._custom_topic, CompressedImage, queue_size=1)

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        
        
        
        # Convert image to grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Set text origin coordinates / position and content
        text_origin = (math.floor((0.1*image.shape[0])), math.floor(0.7*image.shape[1]))
        text = f'Duck {self._vehicle_name} says, \'Cheese! Capturing {gray_image.shape} - quack-tastic!\''

        # printing gray image size since it's single channel and will only print height and width
        print(f'Image size: {gray_image.shape}') 

        # Annotate the gray image
        cv2.putText(gray_image, text, text_origin, cv2.FONT_HERSHEY_DUPLEX, fontScale=0.48, color=(255,255,255), thickness=1)
        
        # display frame
        cv2.imshow(self._window, gray_image)
        cv2.waitKey(1)

        # convert back to ROS message
        annotated_msg = self._bridge.cv2_to_compressed_imgmsg(gray_image)
        annotated_msg.header.stamp = rospy.Time.now()

        # publish annotated image
        self.pub.publish(annotated_msg)
        rospy.loginfo("Published annotated image")

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()