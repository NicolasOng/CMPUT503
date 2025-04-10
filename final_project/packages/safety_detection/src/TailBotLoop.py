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

class TailBot(DTROS):
    def __init__(self, node_name):
        super(TailBot, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # lane following
        self.lane_error = None
        self.lane_error_topic = rospy.Subscriber(f"/{self.vehicle_name}/lane_error", String, self.lane_error_callback)
        self.car_cmd = rospy.Publisher(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)

        # ground color detection
        self.closest_blue = float('inf')
        self.blue_cooldown = 0
        self.color_coords_topic = rospy.Subscriber(f"/{self.vehicle_name}/color_coords", String, self.color_coords_callback)

        # pedestrian detection
        self.pedestrians_detected = False
        self.pedestrians_topic = rospy.Subscriber(f"/{self.vehicle_name}/duckies_info", String, self.pedestrians_callback)

        # other bot detection
        self.other_bot_info = None
        self.other_bot_topic = rospy.Subscriber(f"/{self.vehicle_name}/other_bot_info", String, self.other_bot_callback)

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
        self.closest_blue = min(color_coords["blue"], key=lambda item: item['center'][1])['center'][1] if color_coords["blue"] else float('inf')
    
    def pedestrians_callback(self, msg):
        '''
        pedestrians = {
            "duckie_exist": bool,
            "min_point": float
        }
        '''
        pedestrians_json = msg.data
        self.pedestrians_detected = json.loads(pedestrians_json)["duckie_exist"]
    
    def other_bot_callback(self, msg):
        '''
        other_bot = {
            other_bot_coord: , 
            bot_error: ,
            turning_left: ,
            pixel_distance
        }
        '''
        other_bot_json = msg.data
        self.other_bot_info = json.loads(other_bot_json)
    
    def set_velocities(self, linear, rotational):
        '''
        sets the linear/rotational velocities of the Duckiebot
        linear = m/s
        rotational = radians/s
        '''
        self.car_cmd.publish(Twist2DStamped(v=linear, omega=rotational))
    
    def Tail(self):
        rate_int = 10
        rate = rospy.Rate(rate_int)
        while not rospy.is_shutdown():
            start_time = rospy.Time.now()
            # do the lane following
            v, omega = pid_controller_v_omega(self.lane_error, simple_pid, rate_int, False)
            self.set_velocities(v, omega)

            # stop if the bot is too close to the other bot
            if self.other_bot_info is not None and self.other_bot_info["pixel_distance"] <= 55:
                self.set_velocities(0, 0)
                

            rate.sleep()
            # update the cooldowns
            end_time = rospy.Time.now()
            dt = (end_time - start_time).to_sec()
            self.blue_cooldown = max(0, self.blue_cooldown - dt)

    def on_shutdown(self):
        # on shutdown,
        self.set_velocities(0, 0)
        self.set_velocities(0, 0)
        self.set_velocities(0, 0)
        self.set_velocities(0, 0)

if __name__ == '__main__':
    node = TailBot(node_name='tail')
    rospy.sleep(2)
    node.Tail()
    rospy.spin()
