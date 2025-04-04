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

class PIDController(DTROS):
    def __init__(self, node_name):
        super(PIDController, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # lane error subscriber
        self.lane_error = None
        self.lane_error_topic = rospy.Subscriber(f"/{self.vehicle_name}/lane_error", String, self.lane_error_callback)

        # move command publisher
        self.car_cmd = rospy.Publisher(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)

        # service
        self.service_pid_iteration = rospy.Service(f'/{self.vehicle_name}/pid_iteration', SetString, self.pid_controller_iteration_request)

        # PID controller variables
        self.simple_pid = {
            "kp": -0.025,
            "ki": 0,
            "kd": -0.0125,
            "previous_error": 0,
            "integral": 0
        }

    def lane_error_callback(self, msg):
        '''
        lane_error = {
            "lane_error": error
        }
        '''
        meas_json = msg.data
        self.lane_error = json.loads(meas_json)["lane_error"]
    
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

    def pid_controller_iteration(self, rate, reset=False):
        if self.lane_error is None: return
        dt = 1 / rate
        # get the lane error
        error = self.lane_error
        # feed this into the pid function to get the amount to turn the bot
        # also do clamping
        omega = None
        if error is not None:
            omega = self.get_pid_controls(self.simple_pid, error, dt, reset=reset)
            clamp_value = math.pi * 1
            omega = max(-clamp_value, min(omega, clamp_value))
        rospy.loginfo(f'error: {error}, omega: {omega}')
        # send the calculated omega to the wheel commands
        if error is None:
            self.set_velocities(0, 0)
        else:
            self.set_velocities(0.23, omega)
    
    def pid_controller_iteration_request(self, req):
        params = json.loads(req.data)
        rate, reset = params['rate'], params['reset']
        self.pid_controller_iteration(rate, reset=reset)
        return SetStringResponse(success=True, message=f"Did one iteration of the PID controller with params rate={rate} reset={reset}.")

    def pid_controller(self):
        rate_int = 10
        rate = rospy.Rate(rate_int)
        while not rospy.is_shutdown():
            self.pid_controller_iteration(rate_int)
            rate.sleep()

    def on_shutdown(self):
        # on shutdown,
        self.set_velocities(0, 0)

if __name__ == '__main__':
    node = PIDController(node_name='controller')
    rospy.sleep(2)
    #node.pid_controller()
    rospy.spin()
