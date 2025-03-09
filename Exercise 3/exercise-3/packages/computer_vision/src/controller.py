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
import math

class PIDController(DTROS):
    def __init__(self, node_name):
        super(PIDController, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # maes subscriber
        self.maes = None
        self.camera_sub = rospy.Subscriber(f"/{self.vehicle_name}/maes", String, self.maes_callback)

        # move command publisher
        self.car_cmd = rospy.Publisher(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)

        # PID controller variables
        self.straight_line_pid = {
            "kp": -0.025, #negative,  0.03 too big, 0.01 too small. 0.02~0.025 is good - could keep calibrating though
            "ki": 0,
            "kd": 0, #negative, 0 too small, 0.5 too big
            "previous_error": 0,
            "integral": 0
        }

        self.lane_following_pid = {
            "kp": 0,
            "ki": 0,
            "kd": 0,
            "previous_error": 0,
            "integral": 0
        }

    def maes_callback(self, msg):
        '''
        maes = {
            "yellow": yellow_mae,
            "white": white_mae,
        }
        '''
        meas_json = msg.data
        self.maes = json.loads(meas_json)
    
    def get_pid_controls(self, pid, error, dt, reset=False):
        '''
        The method to get PID controls.
        For P/PD, just set ki and/or kd to 0
        use the reset flag when the desired value changes a lot
        need to tune the kp, ki, kd values for different tasks (keep a note of them)
        '''
        if reset:
            pid['integral'] = 0
            pid['previous_error'] = 0
        # error = desired_value - measured_value
        pid['integral'] += error * dt
        derivative = (error - pid['previous_error']) / dt if dt > 0 else 0
        
        output = (pid['kp'] * error) + (pid['ki'] * pid['integral']) + (pid['kd'] * derivative)
        pid['previous_error'] = error
        
        return output
    
    def set_velocities(self, linear, rotational):
        '''
        sets the linear/rotational velocities of the Duckiebot
        linear = m/s
        rotational = radians/s
        '''
        self.car_cmd.publish(Twist2DStamped(v=linear, omega=rotational))
    
    def straight_line(self):
        rate_int = 5
        rate = rospy.Rate(rate_int)
        while not rospy.is_shutdown():
            if self.maes is None: continue
            dt = 1 / rate_int
            # get the error between yellow line and white line from the camera detection node
            yellow_error, white_error = self.maes["yellow"], self.maes["white"]
            error = None
            if yellow_error is not None and white_error is not None:
                error = (yellow_error + white_error) / 2
            elif yellow_error is None and white_error is not None:
                error = white_error
            elif yellow_error is not None and white_error is None:
                error = yellow_error
            # feed this into the pid function to get the amount to turn the bot
            if error is not None:
                omega = self.get_pid_controls(self.straight_line_pid, error, dt)
                clamp_value = 2 * math.pi
                omega = max(-clamp_value, min(omega, clamp_value))
            rospy.loginfo(f'error: {error}, omega: {omega}')
            # send this to the wheel commands
            if error is None:
                self.set_velocities(0, 0)
            else:
                self.set_velocities(0.25, omega)
            rate.sleep()

    def on_shutdown(self):
        # on shutdown,
        self.set_velocities(0, 0)
        pass

if __name__ == '__main__':
    node = PIDController(node_name='controller')
    rospy.sleep(2)
    node.straight_line()
    rospy.spin()
