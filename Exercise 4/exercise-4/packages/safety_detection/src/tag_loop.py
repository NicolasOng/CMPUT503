#!/usr/bin/env python3
import numpy as np
import json
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Pose2DStamped, WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped, LEDPattern
from safety_detection.srv import SetString, SetStringResponse
from std_msgs.msg import ColorRGBA, String
from Color import Color
import cv2
from cv_bridge import CvBridge
from camera_detection import CameraDetectionNode
import threading
import math

class TagLoop(DTROS):
    def __init__(self, node_name):
        super(TagLoop, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # move node and controller node services
        rospy.wait_for_service(f'/{self.vehicle_name}/rotate')
        rospy.wait_for_service(f'/{self.vehicle_name}/pid_iteration')
        self.rotate_request = rospy.ServiceProxy(f'/{self.vehicle_name}/rotate', SetString)
        self.pid_iteration_request = rospy.ServiceProxy(f'/{self.vehicle_name}/pid_iteration', SetString)

    def pid_iteration(self, rate, reset=False):
        params = {
            "rate": rate,
            "reset": reset
        }
        params_json = json.dumps(params)
        self.pid_iteration_request(params_json)
    
    def tag_loop(self):
        rate_int = 10
        rate = rospy.Rate(rate_int)
        while not rospy.is_shutdown():
            self.pid_iteration(rate_int)
            rate.sleep()

    def on_shutdown(self):
        # on shutdown,
        pass

if __name__ == '__main__':
    node = TagLoop(node_name='tag_loop')
    rospy.sleep(2)
    node.tag_loop()
    rospy.spin()
