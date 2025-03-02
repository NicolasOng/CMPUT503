#!/usr/bin/env python3
import numpy as np
import json
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Pose2DStamped, WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped, LEDPattern
from std_msgs.msg import ColorRGBA, String
from Color import Color
import cv2
from cv_bridge import CvBridge
from move import MoveNode
from camera_detection import CameraDetectionNode
import threading

class PIDController(DTROS):
    def __init__(self, node_name):
        super(PIDController, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # move node
        self.move = None

    def movement(self):
        pass

    def on_shutdown(self):
        # on shutdown,
        pass

if __name__ == '__main__':
    # create the nodes
    camera_detection_node = CameraDetectionNode(node_name='camera_detection_node')
    odometry_node = MoveNode(node_name='odometry_node')
    node = PIDController(node_name='color_based_movement_node')
    node.move = odometry_node
    rospy.sleep(2)
    # start the camera node
    thread1 = threading.Thread(target=camera_detection_node.perform_camera_detection)
    thread1.start()
    # start the move/odometry node
    thread2 = threading.Thread(target=odometry_node.odometry)
    thread2.start()
    # start itself
    node.movement()
    # join the threads
    thread1.join()
    thread2.join()
    rospy.spin()
