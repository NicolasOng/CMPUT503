#!/usr/bin/env python3

import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image

import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
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
        # publisher
        self.undistorted_topic = rospy.Publisher(f"{self._vehicle_name}/undistorted", Image, queue_size=10)
        self.blur_topic = rospy.Publisher(f"{self._vehicle_name}/blur", Image, queue_size=10)
        self.resize_topic = rospy.Publisher(f"{self._vehicle_name}/resize", Image, queue_size=10)

    def callback(self, msg):
        # ******HAVE TO PUBLISH THE IMAGE GENERATED BELOW TO A TOPIC, VIEW IT WITH rqt_image_view
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        # camera matrix and distortion coefficients from intrinsic.yaml file
        cam_matrix = np.array([[319.2461317458548, 0.0, 307.91668484581703], [0.0, 317.75077109798957, 255.6638447529814], [0.0, 0.0, 1.0]])
        dist_coeff = np.array([-0.25706255601943445, 0.045805679651939275, -0.0003584336283982042, -0.0005756902051068707, 0.0])

        # image dimensions
        h,  w = image.shape[:2]
        # optimal camera matrix lets us see entire camera image (image edges cropped without), but some distortion visible
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_matrix, dist_coeff, (w,h), 0, (w,h))

        # undistorted image using calibration parameters
        new_image = cv2.undistort(image, cam_matrix, dist_coeff, None)
        
        # blur
        blur_image = self.blur_image(new_image, kernel_size=5)

        # Resize the image
        resized_image = self.resize_image(blur_image, width=w //2, height=w//2)

        # display frame
        cv2.imshow(self._window, new_image)
        cv2.waitKey(1)

        #publish
        msg_undistorted = self._bridge.cv2_to_imgmsg(new_image, encoding="rgb8")
        self.undistorted_topic.publish(msg_undistorted)

        # publish blur_img
        msg_undistorted_blur = self._bridge.cv2_to_imgmsg(blur_image, encoding="rgb8")
        self.blur_topic.publish(msg_undistorted_blur)

        # publish to resize
        msg_undistorted_blur_resize = self._bridge.cv2_to_imgmsg(resized_image, encoding="rgb8")
        self.resize_topic.publish(msg_undistorted_blur_resize)

    def blur_image(self, img, kernel_size):
        kernel = np.ones((kernel_size,kernel_size),np.float32)/25
        return cv2.blur(img,(kernel_size,kernel_size))

    def resize_image(self, img, width, height):
        return cv2.resize(img, (width, height))
if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    
    # keep spinning
    rospy.spin()
