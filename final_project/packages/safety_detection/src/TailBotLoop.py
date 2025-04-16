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

from pid_controller import simple_pid, pid_controller_v_omega, yellow_white_pid

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
        self.red_cooldown = 0
        self.white_cooldown = 0
        self.color_coords_topic = rospy.Subscriber(f"/{self.vehicle_name}/color_coords", String, self.color_coords_callback)

        # pedestrian detection
        self.pedestrians_detected = False
        self.pedestrians_topic = rospy.Subscriber(f"/{self.vehicle_name}/duckies_info", String, self.pedestrians_callback)

        # other bot detection
        self.other_bot_info = None
        self.other_bot_topic = rospy.Subscriber(f"/{self.vehicle_name}/other_bot_info", String, self.other_bot_callback)


        # service
        self.drive_turn_request = rospy.ServiceProxy(f'/{self.vehicle_name}/drive_turn', SetString)
        self.rotate_request = rospy.ServiceProxy(f'/{self.vehicle_name}/rotate', SetString)

        
        self.closest_red = float('inf')
        self.closest_white = float('inf')

        # tags
        self.tag_list_topic = rospy.Subscriber(f"/{self.vehicle_name}/tag_id", String, self.tag_id_callback)
        self.last_detected_tag_id = -1
        self.current_led_tag_color = -1
        stop_sign_tag_ids = [21, 22, 162, 163]
        intersection_sign_ids = [50, 15, 133, 59, 51, 56]
        ualberta_tag_ids = [94, 93, 200, 201]
        self.none_tag_id = -1
        self.tag_time_dict = {}
        for tag_id in stop_sign_tag_ids:
            self.tag_time_dict[tag_id] = 3
        for tag_id in intersection_sign_ids:
            self.tag_time_dict[tag_id] = 2
        for tag_id in ualberta_tag_ids:
            self.tag_time_dict[tag_id] = 1
        self.tag_time_dict[self.none_tag_id] = 0.5

        # white line management
        self.white_line_on_right = True

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
        self.closest_red = min(color_coords["red"], key=lambda item: item['center'][1])['center'][1] if color_coords["red"] else float('inf')
        self.closest_white = min(color_coords["white"], key=lambda item: item['center'][1])['center'][1] if color_coords["white"] else float('inf')
    
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

    def tag_id_callback(self, msg):
        '''
        msg.data = "id"
        '''
        current_tag = int(msg.data)
        if current_tag != -1:
            self.last_detected_tag_id = current_tag
    
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
        lane_error_valid_before = False  
        while not rospy.is_shutdown():
            if self.lane_error is not None: 
                lane_error_valid_before = True
            start_time = rospy.Time.now()
            # do the lane following
            v, omega = pid_controller_v_omega(self.lane_error, yellow_white_pid, rate_int, False)
            rospy.loginfo(f"lane error: {self.lane_error} v: {v} omega: {omega}")
            self.set_velocities(v, omega)

            # lane_error_valid_before is a 
            # hack for so that bot will not immediately turn right during initialization of the node. since lane_error is None at init
            if self.lane_error is None and lane_error_valid_before: 
                self.drive_turn_right(math.pi / 8, -3.5, 0.2)

            # stop if the bot is too close to the other bot
            if self.other_bot_info is not None and self.other_bot_info["pixel_distance"] <= 55:
                self.set_velocities(0, 0)
            
                
            if self.other_bot_info is not None:
                rospy.loginfo(f"turn left?: {self.other_bot_info['turning_left']} bot error {self.other_bot_info['bot_error']} \
                              pixel distance: {self.other_bot_info['pixel_distance']} omega: {omega} v: {v}; lane error: {self.lane_error}")
                pass
            
            if self.closest_red < 150 and self.red_cooldown == 0 and True:
                self.on_red_line()
                self.hard_code_turning()

            rate.sleep()
            # update the cooldowns
            end_time = rospy.Time.now()
            dt = (end_time - start_time).to_sec()
            self.blue_cooldown = max(0, self.blue_cooldown - dt)
            self.red_cooldown = max(0, self.red_cooldown - dt)
            self.white_cooldown = max(0, self.white_cooldown - dt)

    def hard_code_turning(self):
        '''
        hard code the turning
        '''
        print("hard code turning")
        if self.other_bot_info is not None:
            if self.other_bot_info["turning_left"]:  
                self.drive_turn_left()  # hardcoded left turn
            elif self.other_bot_info["turning_left"] == False:
                self.drive_turn_right(math.pi / 2, -3.5, 0.2)  # hardcoded right turn
                return  # let bot adjust itself  using PID
            elif self.other_bot_info["turning_left"] == None:  # case: other bot isnt turning
                return  # lane follow
            
        else:
            self.drive_turn_right(math.pi / 8, -1, 0.2)
            return
    
    """
    rotate itself
    
    """
    def on_white_line(self):

        return
    
    """
    When self.closest_red < 200, stop the bot and wait for some time
    """
    def on_red_line(self):
        rospy.loginfo(f'detected red line, stopping. tag id: {self.last_detected_tag_id}, time to stop: {self.tag_time_dict[self.last_detected_tag_id]}')
        # update the red cooldown
        self.red_cooldown = 5
        # stop the bot
        self.pause(0.5)
        # wait for some amount of time, depending on the last seen tag id.
        rospy.sleep(self.tag_time_dict[self.last_detected_tag_id])

        # reset the last detected tag id
        self.last_detected_tag_id = self.none_tag_id
        # reset the start time, so time spent waiting is not counted
        start_time = rospy.Time.now()
        rospy.loginfo(f'done red line operations')
        return

    def drive_turn_right(self, angle=math.pi/2, theta=-3.5, speed=0.2):
        rospy.loginfo("Turning right")
        import math
        turn_param = {
            "angle": angle,  # actual angle to turn
            "theta": theta,  # for twisted2D
            "speed": speed,
            "leds": False
        }
        self.drive_turn_request(json.dumps(turn_param))

    def drive_turn_left(self):
        rospy.loginfo("Turning left")
        import math
        turn_param = {
            "angle": math.pi / 1.9,  # actual angle to turn
            "theta": 0.5,  # for twisted2D
            "speed": 0.3,
            "leds": False
        }
        self.drive_turn_request(json.dumps(turn_param))


    def rotate(self, angle, speed):
        '''
        angle is in radians
        speed is in rad/s
        '''
        rotate_param = {
            "radians": angle,
            "speed": speed,
            "leds": False
        }
        self.rotate_request(json.dumps(rotate_param))

    
    def pause(self, seconds):
        '''
        seconds should be positive
        '''
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.set_velocities(0, 0)
            cur_time = rospy.Time.now()
            if (cur_time - start_time).to_sec() >= seconds:
                break
            rate.sleep()

    def on_shutdown(self):
        # on shutdown,
        self.set_velocities(0, 0)

if __name__ == '__main__':
    node = TailBot(node_name='tail')
    rospy.sleep(2)
    #node.drive_turn_right(math.pi / 2, -3.5, 0.2)
    node.Tail()
    #node.drive_turn_left()
    #node.drive_turn_right()
    node.on_shutdown()
    rospy.spin()
