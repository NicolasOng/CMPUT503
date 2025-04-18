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

from pid_controller import simple_pid, yellow_white_pid, bot_following_pid, bot_and_lane_controller

class BotFollowing(DTROS):
    def __init__(self, node_name):
        super(BotFollowing, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # odometry topic
        self.ctheta = 0
        self.cpos = 0
        self.xpos = 0
        self.ypos = 0
        self.lane_error_topic = rospy.Subscriber(f"/{self.vehicle_name}/odometry", String, self.odometry_callback)

        # lane following
        self.lane_error = None
        self.lane_pid_values = simple_pid
        self.lane_error_topic = rospy.Subscriber(f"/{self.vehicle_name}/lane_error", String, self.lane_error_callback)
        self.car_cmd = rospy.Publisher(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)

        # ground color detection
        self.closest_blue = float('inf')
        self.closest_red = float('inf')
        self.red_cooldown = 0
        self.color_coords_topic = rospy.Subscriber(f"/{self.vehicle_name}/color_coords", String, self.color_coords_callback)

        # duckiebot detection
        self.duckiebot_area = 0
        self.bot_error = 0
        self.duckiebot_topic = rospy.Subscriber(f"/{self.vehicle_name}/duckiebot_area", String, self.duckiebot_callback)

    def odometry_callback(self, msg):
        '''
        odometry_data = {
            "cpos": self.cpos,
            "ctheta": self.ctheta,
            ...
        }
        '''
        odometry_data = json.loads(msg.data)
        self.ctheta = odometry_data["ctheta"]
        self.cpos = odometry_data["cpos"]
        self.xpos = odometry_data["xpos"]
        self.ypos = odometry_data["ypos"]

    def lane_error_callback(self, msg):
        '''
        lane_error = {
            "lane_error": error
        }
        '''
        le_json = msg.data
        yellow_lane_error = json.loads(le_json)["yellow_lane_error"]
        white_lane_error = json.loads(le_json)["white_lane_error"]
        self.lane_error = yellow_lane_error
        self.lane_pid_values = yellow_white_pid
        # self.lane_error = white_lane_error
        # self.lane_pid_values = simple_pid

        # if white_lane_error is None:
        #     self.lane_error = yellow_lane_error
        #     self.lane_pid_values = yellow_white_pid
        if yellow_lane_error is None:
            self.lane_error = white_lane_error
            self.lane_pid_values = simple_pid
    
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
        # get the closest red color
        self.closest_red = min(color_coords["red"], key=lambda item: item['center'][1])['center'][1] if color_coords["red"] else float('inf')

    def duckiebot_callback(self, msg):
        '''
        msg = {
            "duckiebot_mask_area": int,
            "contours": [(x, y, w, h), ...],
            "contour_areas": [float, ...]
        }
        '''
        pedestrians_json = msg.data
        self.duckiebot_area = json.loads(pedestrians_json)["duckiebot_mask_area"]
        self.bot_error = 20000 - self.duckiebot_area
        if self.duckiebot_area < 10000:
            self.bot_error = None
    
    def set_velocities(self, linear, rotational):
        '''
        sets the linear/rotational velocities of the Duckiebot
        linear = m/s
        rotational = radians/s
        '''
        self.car_cmd.publish(Twist2DStamped(v=linear, omega=rotational))

    def drive_arc(self, distance, theta, speed=0.23):
        '''
        theta in radians/s, where -1 is left turn, 1 is right turn
        0 is straight
        distance should be positive
        speed should be in [-1, 1]
        '''
        starting_cpos = self.cpos
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.set_velocities(speed, theta)
            cur_meters = self.cpos - starting_cpos
            if cur_meters >= distance:
                break
            rate.sleep()
        self.set_velocities(0, 0)
    
    def bot_following(self):
        rate_int = 10
        rate = rospy.Rate(rate_int)
        while not rospy.is_shutdown():
            start_time = rospy.Time.now()
            # do the bot and lane following
            v, omega = bot_and_lane_controller(self.lane_error, self.bot_error, self.lane_pid_values, bot_following_pid, rate_int, False)
            self.set_velocities(v, omega)
            #rospy.loginfo(f'lane_error: {self.lane_error}, bot error: {self.bot_error}, v: {v}, omega: {omega}')
            rospy.loginfo(f'closest red: {self.closest_red}, red cooldown: {self.red_cooldown}')
            rospy.loginfo(f'xpos: {self.xpos}, ypos: {self.ypos}')
            # if the bot is at a red tape,
            if self.closest_red < 135 and self.red_cooldown == 0:
                rospy.loginfo(f'detected red line, stopping.')
                self.red_cooldown = 5
                # stop the bot
                self.set_velocities(0, 0)
                # wait for 1s,
                rospy.sleep(1)
                # do a left turn
                # 0.2 is good for the top one
                self.drive_arc(0.60, math.pi * 0.27)
                # break
            rate.sleep()
            # update the cooldowns
            end_time = rospy.Time.now()
            dt = (end_time - start_time).to_sec()
            self.red_cooldown = max(0, self.red_cooldown - dt)

    def on_shutdown(self):
        # on shutdown,
        self.set_velocities(0, 0)
        self.set_velocities(0, 0)
        self.set_velocities(0, 0)
        self.set_velocities(0, 0)

if __name__ == '__main__':
    node = BotFollowing(node_name='botfollowing')
    rospy.sleep(2)
    node.bot_following()
    rospy.spin()
