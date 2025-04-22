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

from pid_controller import simple_pid, yellow_white_pid, pid_controller_v_omega

class Pedestrians(DTROS):
    def __init__(self, node_name):
        super(Pedestrians, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # service
        self.service_part_three = rospy.Service(f'/{self.vehicle_name}/part_three', SetString, self.part_three_request)
        self.shutdown_service = rospy.Service(f'/{self.vehicle_name}/part_three_shutdown', SetString, self.shutdown_request)

        # lane following
        self.lane_error = None
        self.pid_values = simple_pid
        self.lane_error_topic = rospy.Subscriber(f"/{self.vehicle_name}/lane_error", String, self.lane_error_callback)
        self.car_cmd = rospy.Publisher(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)

        # ground color detection
        self.closest_blue = float('inf')
        self.closest_red = float('inf')
        self.blue_cooldown = 0
        self.color_coords_topic = rospy.Subscriber(f"/{self.vehicle_name}/color_coords", String, self.color_coords_callback)

        # pedestrian detection
        self.pedestrians_detected = False
        self.pedestrians_topic = rospy.Subscriber(f"/{self.vehicle_name}/duckies_info", String, self.pedestrians_callback)

        # duckiebot detection
        self.duckiebot_area = 0
        self.duckiebot_topic = rospy.Subscriber(f"/{self.vehicle_name}/duckiebot_area", String, self.duckiebot_callback)

        self.blue_count = 0

    def shutdown_request(self, req):
        # req.data = String
        rospy.signal_shutdown("Shut Down Part 3")
        return SetStringResponse(success=True, message=f"Part Three Done!")

    def part_three_request(self, req):
        # req.data = String
        self.pedestrians()
        return SetStringResponse(success=True, message=f"Part Three Done!")

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
        self.pid_values = yellow_white_pid
        # self.lane_error = white_lane_error
        # self.pid_values = simple_pid

        # if white_lane_error is None:
        #     self.lane_error = yellow_lane_error
        #     self.pid_values = yellow_white_pid
        if yellow_lane_error is None:
            self.lane_error = white_lane_error
            self.pid_values = simple_pid
    
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
    
    def pedestrians_callback(self, msg):
        '''
        pedestrians = {
            "duckie_exist": bool,
            "min_point": float
        }
        '''
        pedestrians_json = msg.data
        self.pedestrians_detected = json.loads(pedestrians_json)["duckie_exist"]

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
            start_time = rospy.Time.now()
            # do the lane following
            v, omega = pid_controller_v_omega(self.lane_error, self.pid_values, rate_int, False)
            self.set_velocities(v, omega)
            rospy.loginfo(f'closest blue: {self.closest_blue}, blue cooldown: {self.blue_cooldown}')
            # if the bot is at a blue tape,
            if self.closest_blue < 250 and self.blue_cooldown == 0:
                self.blue_cooldown = 5
                rospy.loginfo(f'detected blue line, stopping for 1s. pedestrians detected: {self.pedestrians_detected}')
                # stop the bot
                self.set_velocities(0, 0)
                # wait for 1s,
                rospy.sleep(3)
                # and continue waiting until no pedestrians are detected
                while self.pedestrians_detected and not rospy.is_shutdown():
                    rospy.loginfo(f'pedestrians detected: {self.pedestrians_detected}')
                    rate.sleep()
                # reset the start time, so time is not counted while waiting for pedestrians
                start_time = rospy.Time.now()

                self.blue_count += 1

            # if the bot is at a duckiebot,
            if self.duckiebot_area > 40000:
                rospy.loginfo(f'duckiebot detected: {self.duckiebot_area}')
                # stop the bot
                self.set_velocities(0, 0)
                # wait for 1s,
                rospy.sleep(3)

            # if the bot is at a red tape,
            if self.closest_red < 200 and self.blue_count == 2: #self.closest_red < 200
                rospy.loginfo(f'detected red line, stopping.')
                # stop the bot
                self.set_velocities(0, 0)
                # wait for 1s,
                rospy.sleep(3)
                rospy.loginfo(f'DONE SECTION3')
                break
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
    node = Pedestrians(node_name='pedestrians')
    #rospy.sleep(2)
    #node.pedestrians()
    rospy.spin()
