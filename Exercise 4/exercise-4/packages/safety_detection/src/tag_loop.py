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

class TagLoop(DTROS):
    def __init__(self, node_name):
        super(TagLoop, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # move node services
        rospy.wait_for_service(f'/{self.vehicle_name}/rotate')
        self.rotate_request = rospy.ServiceProxy(f'/{self.vehicle_name}/rotate', SetString)
        rospy.loginfo('rotate service is setup')

        # lane following
        self.lane_error = None
        self.lane_error_topic = rospy.Subscriber(f"/{self.vehicle_name}/lane_error", String, self.lane_error_callback)
        self.car_cmd = rospy.Publisher(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)

        # tag detection
        self.stop_sign_tag_id = 162
        self.t_intersection_tag_id = 51
        self.ualberta_tag_id = 201
        self.last_detected_tag_id = None
        self.tag_list_topic = rospy.Subscriber(f"/{self.vehicle_name}/tag_id", String, self.tag_list_callback)
        self.tag_time_dict = {
            self.stop_sign_tag_id: 3,
            self.t_intersection_tag_id: 2,
            self.ualberta_tag_id: 1
        }

        # ground color detection
        self.closest_red = None
        self.closest_white = None
        self.color_coords_topic = rospy.Subscriber(f"/{self.vehicle_name}/color_coords", String, self.color_coords_callback)

        # led commands
        self.led_command = rospy.Publisher(f"/{self.vehicle_name}/led_emitter_node/led_pattern", LEDPattern, queue_size=1)
        red = ColorRGBA(r=255, g=0, b=0, a=255)
        white = ColorRGBA(r=255, g=255, b=255, a=255)
        green = ColorRGBA(r=0, g=255, b=0, a=255)
        blue = ColorRGBA(r=0, g=0, b=255, a=255)
        self.default = [white, red, white, red, white]
        self.all_red = [red] * 5
        self.all_green = [green] * 5
        self.all_blue = [blue] * 5
        self.all_white = [white] * 5
        self.tag_to_led = {
            self.stop_sign_tag_id: self.all_red,
            self.t_intersection_tag_id: self.all_blue,
            self.ualberta_tag_id: self.all_green
        }

    def lane_error_callback(self, msg):
        '''
        lane_error = {
            "lane_error": error
        }
        '''
        le_json = msg.data
        self.lane_error = json.loads(le_json)["lane_error"]
    
    def tag_list_callback(self, msg):
        '''
        [
            {
                "id": tag_id,
                "center": [x, y],
                "corners": [[x1, y1], [x2, y2], [x3, y3], [x4, y4]],
                "area": area
            },
            ...
        ]
        '''
        # get the tag list
        tag_list = json.loads(msg.data)
        # get the currently detected tag id with the largest area
        self.last_detected_tag_id = tag_list[0]["id"]
    
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
        # get the closest red color
        self.closest_red = min(color_coords["red"], key=lambda item: item['center'][1]) if color_coords["red"] else None
        # get the closest white color
        self.closest_white = min(color_coords["white"], key=lambda item: item['center'][1]) if color_coords["white"] else None
    
    def rotate(self, radians, speed, leds=False):
        params = {
            "radians": radians,
            "speed": speed,
            "leds": leds
        }
        params_json = json.dumps(params)
        self.rotate_request(params_json)
    
    def set_velocities(self, linear, rotational):
        '''
        sets the linear/rotational velocities of the Duckiebot
        linear = m/s
        rotational = radians/s
        '''
        self.car_cmd.publish(Twist2DStamped(v=linear, omega=rotational))

    def update_leds(self):
        pattern = self.all_white
        if self.last_detected_tag_id:
            pattern = self.tag_to_led[self.last_detected_tag_id]
        self.led_command.publish(LEDPattern(colors=pattern))
    
    def tag_loop(self):
        rate_int = 10
        rate = rospy.Rate(rate_int)
        while not rospy.is_shutdown():
            # update the leds
            self.update_leds()
            # do the lane following
            v, omega = pid_controller_v_omega(self.lane_error, simple_pid, rate_int, False)
            rospy.loginfo(f'error: {self.lane_error}, omega: {omega}')
            self.set_velocities(v, omega)
            # if the bot is at a red tape,
            if self.closest_red['center'] < 20:
                # stop the bot
                self.set_velocities(0, 0)
                # and wait for some amount of time, depending on the last seen tag id.
                wait_time = 0.5
                if self.last_detected_tag_id:
                    wait_time = self.tag_time_dict[self.last_detected_tag_id]
                rospy.sleep(wait_time)
                # and reset the last detected tag id
                self.last_detected_tag_id = None
            # if the bot is at a white tape,
            if self.closest_white['center'] < 20:
                # stop the bot
                self.set_velocities(0, 0)
                # and rotate the bot
                self.rotate_request(math.pi/2, 0.5)
            # TODO: possible pitfall: the bot might stop at the red tap repeatedly - test to see if this happens
            rate.sleep()

    def on_shutdown(self):
        # on shutdown,
        self.set_velocities(0, 0)
        self.led_command.publish(LEDPattern(colors=self.default))

if __name__ == '__main__':
    node = TagLoop(node_name='tag_loop')
    rospy.sleep(2)
    node.tag_loop()
    rospy.spin()
