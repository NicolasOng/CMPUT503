#!/usr/bin/env python3

# import required libraries
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped
import math
import time
import numpy as np

import threading

class MoveNode(DTROS):
    def __init__(self, node_name):
        super(MoveNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # add your code here
        self.vehicle_name = os.environ['VEHICLE_NAME']
        # subscriber callbacks
        # this topic only get published to with keyboard controls, not API controls.
        # need to use wheel encoder ticks
        #self.kinematics_velocity = rospy.Subscriber(f'/{self.vehicle_name}/kinematics_node/velocity', Twist2DStamped, self.velocity_callback)
        self.left_encoder = rospy.Subscriber(f'/{self.vehicle_name}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_wheel_callback)
        self.right_encoder = rospy.Subscriber(f'/{self.vehicle_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_wheel_callback)

        # publishers
        self.wheel_command = rospy.Publisher(f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)

        # LEDs

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
        pass

    def left_wheel_callback(self, msg):
        self.l_res = msg.resolution
        self.l_ticks = msg.data

    def right_wheel_callback(self, msg):  
        self.r_res = msg.resolution 
        self.r_ticks = msg.data

    def calculate_velocities(self):
        rate = rospy.Rate(10)
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

            #dlrads = round(dlrads, 2)
            #drrads = round(drrads, 2)

            #vlticks = dlticks / dtime
            #vrticks = drticks / dtime

            #vlrads = (2 * self.radius * math.pi * vlticks) / self.l_res
            #vrrads = (2 * self.radius * math.pi * vrticks) / self.r_res

            #vlrads = round(vlrads, 2)
            #vrrads = round(vrrads, 2)

            #vpos = (vlrads + vrrads) / 2
            #vrot = (vlrads - vrrads) / (2 * self.w_dist)

            #dpos = vpos * dtime
            #drot = vrot * dtime

            dpos = (self.radius * dlrads + self.radius * drrads) / 2
            drot = (self.radius * dlrads - self.radius * drrads) / (2 * self.w_dist)

            self.xpos = self.xpos + dpos * np.cos(self.theta)
            self.ypos = self.ypos + dpos * np.sin(self.theta)
            self.theta = self.theta + drot

            self.cpos += abs(dpos)
            self.ctheta += abs(drot)
            rospy.loginfo(f"xpos: {self.xpos:.2f}, ypos: {self.ypos:.2f}, theta: {self.theta:.2f}, cpos: {self.cpos:.2f}, ctheta: {self.ctheta:.2f}")

            plticks = self.l_ticks
            prticks = self.r_ticks
            ptime = cur_time
            rate.sleep()
        
    def compute_distance_traveled(self, msg):
        # add your code here
        pass
    
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

    def use_leds(self, **kwargs):
        # add your code here
        pass

    def command_wheel(self, ldirection, lthrottle, rdirection, rthrottle):
        command = WheelsCmdStamped(vel_left=ldirection*lthrottle, vel_right=rdirection*rthrottle)
        self.wheel_command.publish(command)

    # define other functions if needed
    def task1(self):
        #vthread = threading.Thread(target=self.calculate_velocities)
        #vthread.start()
        self.drive_straight(1.25, 0.4)
        self.pause(0.5)
        self.drive_straight(1.25, -0.4)
        #vthread.join()
    
    def task2(self):
        self.rotate(math.pi/2, 0.4)
        self.pause(0.5)
        self.rotate(-math.pi/2, -0.4)
    
    def run(self):
        rospy.sleep(2)  # wait for the node to initialize

        # add your code here
        # call the functions you have defined above for executing the movements
        pass

    def on_shutdown(self):
        self.wheel_command.publish(WheelsCmdStamped(vel_left=0, vel_right=0))

if __name__ == '__main__':
    # define class MoveNode
    node = MoveNode(node_name='move_node')
    rospy.sleep(2)
    node.calculate_velocities()
    #node.task1()
    # call the function run of class MoveNode
    rospy.spin()