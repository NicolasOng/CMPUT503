#!/usr/bin/env python3

# potentially useful for question - 1.1 - 1.4 and 2.1

# import required libraries
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image

import cv2
from cv_bridge import CvBridge

class LaneDetectionNode(DTROS):
    def __init__(self, node_name):
        super(LaneDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # add your code here
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # subscribers
        self.camera_sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        # publishers
        self.undistorted_pub = rospy.Publisher(f"{self._vehicle_name}/undistorted", Image, queue_size=10)
        self.blur_pub = rospy.Publisher(f"{self._vehicle_name}/blur", Image, queue_size=10)
        self.resize_pub = rospy.Publisher(f"{self._vehicle_name}/resize", Image, queue_size=10)

        # camera matrix and distortion coefficients from intrinsic.yaml file
        self.cam_matrix = np.array([[319.2461317458548, 0.0, 307.91668484581703], [0.0, 317.75077109798957, 255.6638447529814], [0.0, 0.0, 1.0]])
        self.dist_coeff = np.array([-0.25706255601943445, 0.045805679651939275, -0.0003584336283982042, -0.0005756902051068707, 0.0])
        
        # color detection parameters in HSV format
        
        # initialize bridge and subscribe to camera feed

        # lane detection publishers

        # LED
        
        # ROI vertices
        
        # define other variables as needed
    
    def undistort_image(self, cv2_img):
        # add your code here
        h,  w = cv2_img.shape[:2]
        # optimal camera matrix lets us see entire camera image (image edges cropped without), but some distortion visible
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.cam_matrix, self.dist_coeff, (w,h), 0, (w,h))

        # undistorted image using calibration parameters
        undistorted_cv2img = cv2.undistort(cv2_img, self.cam_matrix, self.dist_coeff, None)
        return undistorted_cv2img

    def preprocess_image(self, **kwargs):
        # add your code here
        pass
    
    def detect_lane_color(self, **kwargs):
        # add your code here
        pass
    
    def detect_lane(self, **kwargs):
        # add your code here
        # potentially useful in question 2.1
        pass
    
    
    def callback(self, msg):
        # add your code here
        
        # convert compressed image to CV2
        cv2_image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # undistort image
        undistort_cv2_img = self.undistort_image(cv2_image)
        # preprocess image

        # detect lanes - 2.1 
        
        # publish lane detection results
        
        # detect lanes and colors - 1.3
        
        # publish undistorted image
        msg_undistorted = self._bridge.cv2_to_imgmsg(undistort_cv2_img, encoding="rgb8")
        self.undistorted_pub.publish(msg_undistorted)
        
        # control LEDs based on detected colors

        # anything else you want to add here
        
        pass

    # add other functions as needed

if __name__ == '__main__':
    node = LaneDetectionNode(node_name='lane_detection_node')
    rospy.spin()