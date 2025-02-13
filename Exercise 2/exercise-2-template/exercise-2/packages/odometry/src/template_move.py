#!/usr/bin/env python3

# import required libraries
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped, LEDPattern
from std_msgs.msg import ColorRGBA, String
import math
import time
import numpy as np
import json

import threading

class MoveNode(DTROS):
    def __init__(self, node_name):
        super(MoveNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.vehicle_name = os.environ['VEHICLE_NAME']
        # subscriber callbacks
        # /kinematics_node/velocity only get published to with keyboard controls, not API controls.
        # need to use wheel encoder ticks
        #self.kinematics_velocity = rospy.Subscriber(f'/{self.vehicle_name}/kinematics_node/velocity', Twist2DStamped, self.velocity_callback)
        self.left_encoder = rospy.Subscriber(f'/{self.vehicle_name}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_wheel_callback)
        self.right_encoder = rospy.Subscriber(f'/{self.vehicle_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_wheel_callback)

        # publishers
        self.wheel_command = rospy.Publisher(f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.led_command = rospy.Publisher(f"/{self.vehicle_name}/led_emitter_node/led_pattern", LEDPattern, queue_size=1)
        self.odometry_topic = rospy.Publisher(f'/{self.vehicle_name}/exercise2/odometry', String, queue_size=10)

        # variables for kinematics/velocity
        self.xpos = 0
        self.ypos = 0
        self.theta = 0
        self.ctheta = 0
        self.cpos = 0

        # variables for ticks
        self.radius = rospy.get_param(f'/{self.vehicle_name}/kinematics_node/radius', 0.0325)
        self.circumference = 2 * math.pi * self.radius
        self.w_dist = rospy.get_param(f'/{self.vehicle_name}/kinematics_node/baseline', 0.1) / 2
        self.l_res = -1
        self.r_res = -1
        self.l_ticks = -1
        self.r_ticks = -1

        # corrective values
        self.rot_correction = 1.75
        self.pos_correction = 1.5

    def left_wheel_callback(self, msg):
        self.l_res = msg.resolution
        self.l_ticks = msg.data

    def right_wheel_callback(self, msg):  
        self.r_res = msg.resolution 
        self.r_ticks = msg.data

    def calculate_velocities(self):
        rate = rospy.Rate(5)
        ptime = rospy.Time.now()
        plticks = -1
        prticks = -1

        while plticks == -1 or prticks == -1:
            plticks = self.l_ticks
            prticks = self.r_ticks
            ptime = rospy.Time.now()
            rate.sleep()

        while not rospy.is_shutdown():
            cur_time = rospy.Time.now()

            dtime = cur_time - ptime
            dtime = dtime.to_sec()

            dlticks = plticks - self.l_ticks
            drticks = prticks - self.r_ticks

            rospy.loginfo(f'left: {dlticks}, right: {drticks}')

            dlrads = (dlticks / self.l_res) * (2 * math.pi)
            drrads = (drticks / self.r_res) * (2 * math.pi)

            dpos = (self.radius * dlrads + self.radius * drrads) / 2
            drot = (self.radius * dlrads - self.radius * drrads) / (2 * self.w_dist)

            drot = drot * self.rot_correction
            dpos = dpos * self.pos_correction

            self.xpos = self.xpos + dpos * np.cos(self.theta)
            self.ypos = self.ypos + dpos * np.sin(self.theta)
            self.theta = self.theta + drot

            self.cpos += abs(dpos)
            self.ctheta += abs(drot)
            rospy.loginfo(f"xpos: {self.xpos:.2f}, ypos: {self.ypos:.2f}, theta: {self.theta:.2f}, cpos: {self.cpos:.2f}, ctheta: {self.ctheta:.2f}")
            
            odometry_data = {
                "time": cur_time.to_sec(),
                "interval": dtime,
                "xpos": self.xpos,
                "ypos": self.ypos,
                "theta": self.theta,
                "cpos": self.cpos,
                "ctheta": self.ctheta
            }
            json_odometry = json.dumps(odometry_data)
            self.odometry_topic.publish(json_odometry)

            plticks = self.l_ticks
            prticks = self.r_ticks
            ptime = cur_time
            rate.sleep()
    
    def drive_straight(self, meters, speed):
        '''
        meters should be positive
        speed can be positive for forwards,
        negative for backwards
        '''
        starting_cpos = self.cpos
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo(f"cur_meters: {self.cpos}")
            self.command_wheel(1, speed, 1, speed)
            cur_meters = self.cpos - starting_cpos
            rospy.loginfo(f"cur_meters: {self.cpos}")
            if cur_meters >= meters:
                break
            rate.sleep()
        self.command_wheel(0, 0, 0, 0)
    
    def rotate(self, radians, speed):
        '''
        radians should be positive.
        speed can be positive for clockwise,
        negative for counter-clockwise
        '''
        starting_ctheta = self.ctheta
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.command_wheel(1, speed, -1, speed)
            cur_radians = self.ctheta - starting_ctheta
            if cur_radians >= radians:
                break
            rate.sleep()
        self.command_wheel(0, 0, 0, 0)
        
    def drive_arc(self, distance, arc, speed):
        '''
        arc in [-1, 1], where -1 is full left turn, 1 is full right turn
        0 is straight
        distance should be positive
        speed should be in [-1, 1]
        '''
        starting_cpos = self.cpos
        left_speed = speed * (1 + arc)
        right_speed = speed * (1 - arc)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("arc")
            self.command_wheel(1, left_speed, 1, right_speed)
            cur_meters = self.cpos - starting_cpos
            if cur_meters >= distance:
                break
            rate.sleep()
        self.command_wheel(0, 0, 0, 0)

    def pause(self, seconds):
        '''
        seconds should be positive
        '''
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.command_wheel(0, 0, 0, 0)
            cur_time = rospy.Time.now()
            if (cur_time - start_time).to_sec() >= seconds:
                break
            rate.sleep()

    def command_leds_test(self):
        '''
        sets the leds to the set colors below
        this method mostly used to test LED functionality
        Header header
        string[]  color_list
        std_msgs/ColorRGBA[]  rgb_vals
        int8[]    color_mask
        float32   frequency
        int8[]    frequency_mask
        LEDs:
        - 0: front, port side
        - 1: back, fan side
        - 2: ???
        - 3: back, port side
        - 4: front, fan side
        '''
        command = LEDPattern()
        purple = ColorRGBA(r=255, g=0, b=255, a=255)
        red = ColorRGBA(r=255, g=0, b=0, a=255)
        green = ColorRGBA(r=0, g=255, b=0, a=255)
        blue = ColorRGBA(r=0, g=0, b=255, a=255)
        cyan = ColorRGBA(r=0, g=255, b=255, a=255)
        command.rgb_vals = [purple, purple, green, purple, purple]
        self.led_command.publish(command)

    def command_leds_color(self, color=ColorRGBA(r=255, g=255, b=255, a=255)):
        '''
        sets all the leds to the given color
        '''
        command = LEDPattern()
        command.rgb_vals = [color] * 5
        self.led_command.publish(command)
    
    def command_leds_default(self):
        '''
        set the leds back to default colors
        '''
        command = LEDPattern()
        white = ColorRGBA(r=255, g=255, b=255, a=255)
        red = ColorRGBA(r=255, g=0, b=0, a=255)
        command.rgb_vals = [white, red, white, red, white]
        self.led_command.publish(command)

    def command_wheel(self, ldirection, lthrottle, rdirection, rthrottle):
        '''
        sets the wheel velocities
        '''
        command = WheelsCmdStamped(vel_left=ldirection*lthrottle, vel_right=rdirection*rthrottle)
        self.wheel_command.publish(command)

    # define other functions if needed
    def straight_line_task(self):
        self.drive_straight(1.25, 0.4)
        self.pause(0.5)
        self.drive_straight(1.25, -0.4)
        self.pause(0.5)
    
    def rotation_task(self):
        self.rotate(math.pi/2, 0.4)
        self.pause(0.5)
        self.rotate(math.pi/2, -0.4)
        self.pause(0.5)

    def d_task(self):
        pt = 2
        # stop led signal + pause
        self.command_leds_color(ColorRGBA(r=255, g=0, b=0, a=255))
        self.pause(5)
        # start led signal
        self.command_leds_color(ColorRGBA(r=0, g=255, b=0, a=255))
        # long straight
        self.drive_arc(1.1, 0.05, 0.4)
        self.pause(pt)
        # first corner
        self.rotate(math.pi/2, 0.3)
        self.pause(pt)
        # first short straight
        self.drive_arc(0.75, 0.1, 0.4)
        self.pause(pt)
        # first arc
        self.drive_arc(0.6, 0.4, 0.5)
        self.pause(pt)
        # second short straight
        self.drive_arc(0.4, 0.1, 0.4)
        self.pause(pt)
        # second arc
        self.drive_arc(0.62, 0.4, 0.5)
        self.pause(pt)
        # second short straight
        self.drive_arc(0.6, 0.075, 0.4)
        self.pause(pt)
        # second corner
        self.rotate(math.pi/2, 0.3)
        self.pause(pt)
        # stop led signal + pause 2
        self.command_leds_color(ColorRGBA(r=255, g=0, b=0, a=255))
        self.pause(5)

    def on_shutdown(self):
        # on shutdown,
        # stop the wheels
        self.wheel_command.publish(WheelsCmdStamped(vel_left=0, vel_right=0))
        # reset the leds
        self.command_leds_default()

if __name__ == '__main__':
    # create node
    node = MoveNode(node_name='move_node')
    # wait for it to initialize
    rospy.sleep(2)
    # start the thread that calculates odometry
    vthread = threading.Thread(target=node.calculate_velocities)
    vthread.start()
    # start the d task
    node.d_task()
    # join the odometry thread (thread ends on shutdown)
    vthread.join()
    rospy.spin()
