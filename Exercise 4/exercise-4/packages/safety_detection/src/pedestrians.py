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

from pid_controller import simple_pid, pid_controller_v_omega

class Pedestrians(DTROS):
    def __init__(self, node_name):
        super(Pedestrians, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # lane following
        self.lane_error = None
        self.lane_error_topic = rospy.Subscriber(f"/{self.vehicle_name}/lane_error", String, self.lane_error_callback)
        self.car_cmd = rospy.Publisher(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)

        # ground color detection
        self.closest_blue = None
        self.color_coords_topic = rospy.Subscriber(f"/{self.vehicle_name}/color_coords", String, self.color_coords_callback)

        # TODO: pedestrian detection
        self.pedestrians_detected = False

    def lane_error_callback(self, msg):
        '''
        lane_error = {
            "lane_error": error
        }
        '''
        le_json = msg.data
        self.lane_error = json.loads(le_json)["lane_error"]
    
    def color_coords_callback(self, msg):
        '''
        color_coords = {
            "red": [
                {
                'bb': [x, y, w, h],
                'center': (x, y)
                },
                ...
            ],
            "white": ...,
            "blue": ...
        }
        '''
        # get the color coords
        color_coords = json.loads(msg.data)
        # get the closest blue color
        self.closest_blue = min(color_coords["blue"], key=lambda item: item['center'][1]) if color_coords["blue"] else None
    
    def set_velocities(self, linear, rotational):
        '''
        sets the linear/rotational velocities of the Duckiebot
        linear = m/s
        rotational = radians/s
        '''
        self.car_cmd.publish(Twist2DStamped(v=linear, omega=rotational))
    
    def pedestrians(self):
        rate_int = 10
        rate = rospy.Rate(rate_int)
        while not rospy.is_shutdown():
            # do the lane following
            v, omega = pid_controller_v_omega(self.lane_error, simple_pid, rate_int, False)
            rospy.loginfo(f'error: {self.lane_error}, omega: {omega}')
            self.set_velocities(v, omega)
            # if the bot is at a blue tape,
            if self.closest_blue['center'] < 20:
                # stop the bot
                self.set_velocities(0, 0)
                # wait for 1s,
                rospy.sleep(1)
                # and continue waiting until no pedestrians are detected
                while self.pedestrians_detected:
                    rate.sleep()
            # TODO: possible pitfall: the bot might stop at the blue tap repeatedly - test to see if this happens
            rate.sleep()

    def on_shutdown(self):
        # on shutdown,
        self.set_velocities(0, 0)

if __name__ == '__main__':
    node = Pedestrians(node_name='pedestrians')
    rospy.sleep(2)
    node.pedestrians()
    rospy.spin()
