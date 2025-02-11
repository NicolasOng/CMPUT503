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
        # kinematics not working????
        self.kinematics_velocity = rospy.Subscriber(f'/{self.vehicle_name}/kinematics_node/velocity', Twist2DStamped, self.velocity_callback)
        #self.left_encoder = rospy.Subscriber(f'/{self.vehicle_name}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_wheel_callback)
        #self.right_encoder = rospy.Subscriber(f'/{self.vehicle_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_wheel_callback)

        # publishers
        self.wheel_command = rospy.Publisher(f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)

        # LEDs

        # variables for kinematics/velocity
        self.prev_time = -1
        self.xpos = 0
        self.ypos = 0
        self.theta = 0
        self.ctheta = 0
        self.cpos = 0

        # variables for ticks
        self.radius = rospy.get_param(f'/{self.vehicle_name}/kinematics_node/radius', 0.0325)
        self.w_dist = rospy.get_param(f'/{self.vehicle_name}/kinematics_node/baseline', 0.1)
        self.l_res = -1
        self.r_res = -1
        self.l_time = -1
        self.r_time = -1
        self.l_ticks = -1
        self.r_ticks = -1
        self.l_vrads = 0
        self.r_vrads = 0
        pass
        
    def velocity_callback(self, msg):
        # add your code here
        # can define one or two depending on how you want to implement
        rospy.loginfo(f'time: {msg.header.stamp}, v: {msg.v}, omega: {msg.omega}')
        if self.prev_time == -1:
            self.prev_time = msg.header.stamp
            return
        vpos = msg.v
        vrot = msg.omega
        cur_time = msg.header.stamp
        dtime = cur_time - self.prev_time

        dpos = vpos * dtime
        drot = vrot * dtime

        self.xpos = self.xpos + dpos * np.cos(self.theta)
        self.ypos = self.ypos + dpos * np.sin(self.theta)
        self.theta = self.theta + drot

        self.cpos += dpos
        self.ctheta += drot

        rospy.loginfo(f"xpos: {self.xpos}, ypos: {self.ypos}, theta: {self.theta}, cpos: {self.cpos}, ctheta: {self.ctheta}")

    def left_wheel_callback(self, msg):
        # add your code here
        # can define one or two depending on how you want to implement 
        #rospy.loginfo(f"left_wheel: {msg.data}")
        self.l_res = msg.resolution 
        if self.l_ticks == -1:
            self.l_ticks = msg.data
            self.l_time = msg.header.stamp
            return
        
        cur_time = msg.header.stamp
        cur_ticks = msg.data

        '''
        dtime = cur_time - self.l_time
        dticks = cur_ticks - self.l_ticks
        vticks = dticks / dtime.to_sec()
        vrads = (2 * self.radius * math.pi * vticks) / self.l_res
        '''

        self.l_time = cur_time
        self.l_ticks = cur_ticks
        #self.l_vrads = round(vrads, 2)

    def right_wheel_callback(self, msg):
        # add your code here
        # can define one or two depending on how you want to implement  
        #rospy.loginfo(f"right_wheel: {msg.data}")    
        self.r_res = msg.resolution 
        if self.r_ticks == -1:
            self.r_ticks = msg.data
            self.r_time = msg.header.stamp
            return
        
        cur_time = msg.header.stamp
        cur_ticks = msg.data

        '''
        dtime = cur_time - self.r_time
        dticks = cur_ticks - self.r_ticks
        vticks = dticks / dtime.to_sec()
        vrads = (2 * self.radius * math.pi * vticks) / self.r_res
        '''

        self.r_time = cur_time
        self.r_ticks = cur_ticks
        #self.r_vrads = round(vrads, 2)

    def calculate_velocities(self):
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        ptime = rospy.Time.now()
        plticks = -1
        prticks = -1
        while plticks == -1 or prticks == -1:
            plticks = self.l_ticks
            prticks = self.r_ticks
            ptime = rospy.Time.now()
            rate.sleep()

        while not rospy.is_shutdown():
            rospy.loginfo(f'left: {self.l_ticks}, right: {self.l_ticks}')

            cur_time = rospy.Time.now()

            dtime = cur_time - ptime

            dlticks = plticks - self.l_ticks
            drticks = prticks - self.r_ticks

            vlticks = dlticks / dtime.to_sec()
            vrticks = drticks / dtime.to_sec()

            vlrads = (2 * self.radius * math.pi * vlticks) / self.l_res
            vrrads = (2 * self.radius * math.pi * vrticks) / self.r_res

            vlrads = round(vlrads, 2)
            vrrads = round(vrrads, 2)

            vpos = (vlrads + vrrads) / 2
            vrot = (vlrads + vrrads) / (2 * self.w_dist)
            dtime = cur_time - start_time
            dtime = dtime.to_sec()

            dpos = vpos * dtime
            drot = vrot * dtime

            self.xpos = self.xpos + dpos * np.cos(self.theta)
            self.ypos = self.ypos + dpos * np.sin(self.theta)
            self.theta = self.theta + drot

            self.cpos += abs(dpos)
            self.ctheta += abs(drot)
            rospy.loginfo(f"xpos: {self.xpos}, ypos: {self.ypos}, theta: {self.theta}, cpos: {self.cpos}, ctheta: {self.ctheta}")

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
    #node.calculate_velocities()
    node.task1()
    # call the function run of class MoveNode
    rospy.spin()