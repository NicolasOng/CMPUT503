#!/usr/bin/env python3

# import required libraries

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):
    def __init__(self, node_name):
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        # add your code here

        # subscribe to the camera topic
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        # publish to the annotated image topic
        #self._publisher = rospy.Publisher(f"/{self._vehicle_name}/camera_node/image/gray_scale", CompressedImage, queue_size=10)
        self._publisher = rospy.Publisher(self._camera_topic, CompressedImage, queue_size=10)
        pass

    def callback(self, msg):
        # add your code here

        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        print(f'normal_img {image.shape} ')
        # convert to grayscale
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray_image = np.stack((gray_image,)*3, axis=-1)

        size = image.shape[0:2]
        text = f"Duck {self._vehicle_name} says, ‘Cheese! Capturing {size} – quack-tastic!’"

        cv2.putText(gray_image, text, (0, size[1] // 2), fontFace = cv2.FONT_HERSHEY_COMPLEX, fontScale = 0.5, color=(250, 225, 100))

        compressed_image_msg = self._bridge.cv2_to_compressed_imgmsg(gray_image, dst_format="jpg")
        print(f'gray_img {gray_image.shape} {type(gray_image)}')
        # annotate the image
        # publish annotated grayscale image
        if not rospy.is_shutdown():
            rospy.loginfo("Publishing message")
            self._publisher.publish(compressed_image_msg)


    # define other functions if needed

if __name__ == '__main__':
    # define class CameraReaderNode
    # create the node
    node = CameraReaderNode(node_name='camera_annotate_node')
    # run node
    # call the function run of class CameraReaderNode
    rospy.spin()
