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
#from std_srvs.srv import SetString
from computer_vision.srv import SetString, SetStringResponse
import math

class ColorBasedMovement(DTROS):
    def __init__(self, node_name):
        super(ColorBasedMovement, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # move node services
        rospy.wait_for_service(f'/{self.vehicle_name}/drive_straight')
        rospy.wait_for_service(f'/{self.vehicle_name}/rotate')
        rospy.wait_for_service(f'/{self.vehicle_name}/drive_arc')
        rospy.wait_for_service(f'/{self.vehicle_name}/pause')
        self.drive_straight_request = rospy.ServiceProxy(f'/{self.vehicle_name}/drive_straight', SetString)
        self.rotate_request = rospy.ServiceProxy(f'/{self.vehicle_name}/rotate', SetString)
        self.drive_arc_request = rospy.ServiceProxy(f'/{self.vehicle_name}/drive_arc', SetString)
        self.pause_request = rospy.ServiceProxy(f'/{self.vehicle_name}/pause', SetString)
        self.drive_turn_request = rospy.ServiceProxy(f'/{self.vehicle_name}/drive_turn', SetString)
        
        # car command publisher
        self.car_cmd = rospy.Publisher(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        # led publisher
        self.led_command = rospy.Publisher(f"/{self.vehicle_name}/led_emitter_node/led_pattern", LEDPattern, queue_size=1)

        # color coords subscriber
        self.color_coords_topic = rospy.Subscriber(f"/{self.vehicle_name}/color_coords", String, self.color_coords_callback)
        self.color_coords = None
        self.stop_distance = 150 # in mm

        # led commands
        self.white = ColorRGBA(r=255, g=255, b=255, a=255)
        self.red = ColorRGBA(r=255, g=0, b=0, a=255)
        '''
        - 0: front, port side
        - 1: back, fan side
        - 2: ???
        - 3: back, port side
        - 4: front, fan side
        '''
        self.indicate_left_list = [self.red, self.white, self.white, self.red, self.white]
        self.indicate_right_list = [self.white, self.red, self.white, self.white, self.red]
        self.all_white_list = [self.white]*5
        self.all_red_list = [self.red]*5
        self.indicate_left = LEDPattern(rgb_vals=self.indicate_left_list)
        self.indicate_right = LEDPattern(rgb_vals=self.indicate_right_list)
        self.all_white = LEDPattern(rgb_vals=self.all_white_list)
        self.all_red = LEDPattern(rgb_vals=self.all_red_list)

    
    def color_coords_callback(self, msg):
        '''
        color_coords = {
            "red": red_coords,
            "blue": blue_coords,
            "green": green_coords
        }
        '''
        self.color_coords = json.loads(msg.data)

    def drive_straight(self, meters, speed, leds=False):
        params = {
            "meters": meters,
            "speed": speed,
            "leds": leds
        }
        params_json = json.dumps(params)
        self.drive_straight_request(params_json)
    
    def rotate(self, radians, speed, leds=False):
        params = {
            "radians": radians,
            "speed": speed,
            "leds": leds
        }
        params_json = json.dumps(params)
        self.rotate_request(params_json)
        
    def drive_arc(self, distance, theta, speed, leds=False):
        params = {
            "distance": distance,
            "theta": theta,
            "speed": speed,
            "leds": leds
        }
        params_json = json.dumps(params)
        self.drive_arc_request(params_json)

    def drive_turn(self, angle, theta, speed, leds=False):
        params = {
            "angle": angle,
            "theta": theta,
            "speed": speed,
            "leds": leds
        }
        params_json = json.dumps(params)
        self.drive_turn_request(params_json)

    def pause(self, seconds, leds=False):
        params = {
            "seconds": seconds,
            "leds": leds
        }
        params_json = json.dumps(params)
        self.pause_request(params_json)

    def blink_left(self):
        n = 5
        for _ in range(n):
            self.led_command.publish(self.all_white)
            rospy.sleep(0.5)
            self.led_command.publish(self.indicate_left)
            rospy.sleep(0.5)

    def blink_right(self):
        n = 5
        for _ in range(n):
            self.led_command.publish(self.all_white)
            rospy.sleep(0.5)
            self.led_command.publish(self.indicate_right)
            rospy.sleep(0.5)
    
    def all_blink(self):
        n = 5
        for _ in range(n):
            self.led_command.publish(self.all_white)
            rospy.sleep(0.5)
            self.led_command.publish(self.all_red)
            rospy.sleep(0.5)

    def on_red(self):
        # indicate all lights
        self.all_blink()
        # move straight for at least 30cm
        self.drive_straight(0.5, 0.4, True)
        # pause
        self.pause(1)

    def on_blue(self):
        # signals on the right side
        self.blink_right()
        # move in a curve through 90 degrees to the right
        self.drive_arc(0.2, -math.pi, 0.25, True)
        # pause
        self.pause(1)

    def on_green(self):
        # signals on the left side
        self.blink_left()
        # move i a curve through 90 degrees to the left
        self.drive_arc(0.5, math.pi * 0.5, 0.25, True)
        # pause
        self.pause(1)

    def movement(self):
        # drive straight until it hits a r/g/b line
        self.car_cmd.publish(Twist2DStamped(v=0, omega=0))
        no_line = True
        line_color = None
        rate = rospy.Rate(10)
        while no_line:
            if self.color_coords is None: continue
            # get the distance the detected colors are from the bot
            red_y, blue_y, green_y = self.color_coords['red'][1], self.color_coords['blue'][1], self.color_coords['green'][1]
            rospy.loginfo(f'{red_y:.2f}, {blue_y:.2f}, {green_y:.2f}')
            # see if any of the colors are detected close to the bot
            if red_y > 0 and red_y < self.stop_distance: line_color = "red"
            if blue_y > 0 and blue_y < self.stop_distance: line_color = "blue"
            if green_y > 0 and green_y < self.stop_distance: line_color = "green"
            # if so, break
            if line_color is not None: break
            # otherwise, keep driving straight
            self.car_cmd.publish(Twist2DStamped(v=0.2, omega=0))
            rate.sleep()
        # stop before the line for 3-5 seconds (it also blinks, which adds some time)
        self.pause(1, True)
        # perform the line-specific actions
        if line_color == "red":
            rospy.loginfo(f'on red')
            self.on_red()
        elif line_color == "blue":
            rospy.loginfo(f'on blue')
            self.on_blue()
        elif line_color == "green":
            rospy.loginfo(f'on green')
            self.on_green()

    def on_shutdown(self):
        # on shutdown,
        self.car_cmd.publish(Twist2DStamped(v=0, omega=0))
        pass

if __name__ == '__main__':
    node = ColorBasedMovement(node_name='color_based_movement_node')
    rospy.sleep(2)
    node.movement()
    rospy.spin()
