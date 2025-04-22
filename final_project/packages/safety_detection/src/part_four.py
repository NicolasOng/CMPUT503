#!/usr/bin/env python3
import json
import math
import os
import random

import cv2
from cv_bridge import CvBridge
import dt_apriltags
import numpy as np

import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Pose2DStamped, WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped, LEDPattern
from std_msgs.msg import ColorRGBA, String
from safety_detection.srv import SetString, SetStringResponse

from camera_detection import CameraDetectionNode
from Color import Color
from pid_controller import parking_pid, pid_controller_v_omega
from safety_detection.srv import SetString, SetStringResponse

class Parking(DTROS):
    def __init__(self, node_name):
        super(Parking, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # service
        self.service_part_four = rospy.Service(f'/{self.vehicle_name}/part_four', SetString, self.part_four_request)
        self.shutdown_service = rospy.Service(f'/{self.vehicle_name}/part_four_shutdown', SetString, self.shutdown_request)

         # camera subscriber
        self.camera_image = None
        self.bridge = CvBridge()
        self.camera_sub = rospy.Subscriber(f"/{self.vehicle_name}/camera_node/image/compressed", CompressedImage, self.camera_callback)
        self.camera_sub

        # odometry topic
        self.ctheta = 0
        self.cpos = 0
        self.lane_error_topic = rospy.Subscriber(f"/{self.vehicle_name}/odometry", String, self.odometry_callback)

        # vehicle control
        self.car_cmd = rospy.Publisher(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)

        # tag detection
        self.draw_atag_toggle = True
        self.parking_tag = 47
        self.is_ToI = False
        self.ToI_area = -1
        self.ToI_error =  0 
        self.at_detector = dt_apriltags.Detector()
        self.tag_image_sub = rospy.Publisher(f"/{self.vehicle_name}/tag_image_new", Image, queue_size=1)

        # course control
        self.is_start = True
        self.lost_tag = False

        # Parking spot IDs and corresponding variables for hard-coded maneuvers
        self.fixed_maneuvers = {47:(0.450, 1),     58:(0.450, -1),      # 4 ID=47     2 ID=58
                                13:(0.225, 1),     44:(0.225, -1)}      # 3 ID=13     1 ID=44
        
        self.arcs = {47:(0.6, math.pi * 0.35),   58:(0.6, -math.pi * 0.35),
                     13:(0.5, math.pi * 1.2),    44:(0.5, -math.pi * 1.2)}
        

        # LEDs
        self.led_command = rospy.Publisher(f"/{self.vehicle_name}/led_emitter_node/led_pattern", LEDPattern, queue_size=1)
        red = ColorRGBA(r=255, g=0, b=0, a=255)
        white = ColorRGBA(r=255, g=255, b=255, a=255)
        self. default_list = [white, red, white, red, white]
        self.default = LEDPattern(rgb_vals=self.default_list)

        # camera matrix and distortion coefficients from intrinsic.yaml file
        self.cam_matrix = np.array([
            [319.2461317458548, 0.0, 307.91668484581703],
            [0.0, 317.75077109798957, 255.6638447529814],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeff = np.array([-0.25706255601943445, 0.045805679651939275, -0.0003584336283982042, -0.0005756902051068707, 0.0])

    def shutdown_request(self, req):
        # req.data = String
        rospy.signal_shutdown("Shut Down Part 4")
        return SetStringResponse(success=True, message=f"Part Four Done!")

    def part_four_request(self, req):
        # req.data = String
        self.parking()
        return SetStringResponse(success=True, message=f"Part Four Done!")
    

    def camera_callback(self, msg):
        # convert compressed image to cv2
        cv2_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        # save the raw camera image
        self.camera_image = cv2_image


    def undistort_image(self, cv2_img):
        h, w = cv2_img.shape[:2]
        # optimal camera matrix lets us see entire camera image (image edges cropped without), but some distortion visible
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.cam_matrix, self.dist_coeff, (w,h), 0, (w,h))

        # undistorted image using calibration parameters
        return cv2.undistort(cv2_img, self.cam_matrix, self.dist_coeff, None)
    

    def lane_error_callback(self, msg):
        '''
        lane_error = {
            "lane_error": error
        }
        '''
        le_json = msg.data
        self.lane_error = json.loads(le_json)["lane_error"]

    
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
        #rospy.loginfo(f'linear: {linear}, omega: {rotational}')


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


    def rotate(self, radians, speed=0.23):
        '''
        radians should be positive.
        speed can be positive for clockwise,
        negative for counter-clockwise
        '''
        starting_ctheta = self.ctheta
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.set_velocities(0, speed)
            cur_radians = self.ctheta - starting_ctheta
            if cur_radians >= radians:
                break
            rate.sleep()
        self.set_velocities(0, 0)


    def drive_straight(self, distance, speed=0.23):
        '''
        0 is straight
        distance should be positive
        speed should be in [-1, 1]
        '''
        starting_cpos = self.cpos
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.set_velocities(speed, 0)
            cur_meters = self.cpos - starting_cpos
            print("Distance travelled: ", cur_meters)
            if cur_meters >= distance:
                print("stopping")
                break
            rate.sleep()
        self.set_velocities(0, 0)

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


    # Draws a bounding box and ID on an ApriltTag 
    def draw_atag_features(self, image, points, id, center, error, colour=(255, 100, 255)):
        h, w = image.shape[:2]
        img = cv2.polylines(image, [points], True, colour, 5)
        img = cv2.putText(image, error, tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 0.75, colour, 1)
        img = cv2.line(image, (w//2, h//2), tuple(center), colour, 2)
        
        # Image center
        img = cv2.line(image, (w//2, 0), (w//2, h), (255, 100, 255), 2)
        img = cv2.circle(image, (w//2, h//2), 3, (255, 100, 255), 3)
    
        #img = cv2.putText(image, id, (center[0], center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.75, colour, 2)
        return img
    

    def perform_tag_detection(self, clean_image, draw_image):
        # Preprocess image
        image_grey = cv2.cvtColor(clean_image, cv2.COLOR_BGR2GRAY)
        image_grey = cv2.GaussianBlur(image_grey, (5,5), 0)
        
        # ApriltTag detector
        results = self.at_detector.detect(image_grey)

        ToI_index = -1
        self.is_ToI = False
        self.ToI_area = 0
        self.ToI_error = 0

        if len(results) == 0:
            self.tag_image_sub.publish(self.bridge.cv2_to_imgmsg(draw_image, encoding="bgr8"))
            return draw_image
        else:
            for idx, r in enumerate(results):
                if r.tag_id == self.parking_tag:
                    ToI_index = idx
                    self.is_ToI = True

        if ToI_index != -1:
            ToI = results[ToI_index]
            ToI_center = ToI.center.astype(int)
            ToI_corners = np.array(ToI.corners, np.int32)
            ToI_corners = ToI_corners.reshape((-1, 1, 2))
            ToI_id = str(ToI.tag_id)

            self.tag_center = ToI_center

            br = ToI.corners[1].astype(int) # br is w,h
            tl = ToI.corners[3].astype(int) # tl is 0,0
            ToI_area = (br[0] - tl[0]) * (br[1] - tl[1])
            self.ToI_area = ToI_area

            _, w = draw_image.shape[:2]
            ToI_offset_error = ToI_center[0] - w//2
            self.ToI_error = ToI_offset_error

            draw_image = self.draw_atag_features(draw_image, ToI_corners, ToI_id, ToI_center, str(ToI_offset_error))

        self.tag_image_sub.publish(self.bridge.cv2_to_imgmsg(draw_image, encoding="bgr8"))

        return draw_image


    def parking(self):
        if rospy.has_param('p_parking_spot'):
            if int(rospy.get_param('p_parking_spot')) in self.fixed_maneuvers:   
                rospy.loginfo(f"Setting park spot")
                self.parking_tag = int(rospy.get_param('p_parking_spot'))
        else:
            rospy.loginfo(f"Parking spot parameter not found or is an invalid value. Using default (13)")
            self.parking_tag = 13

        rate_int = 10
        rate = rospy.Rate(rate_int)
        while not rospy.is_shutdown():
            clean_image = self.camera_image.copy()
            # undistort camera image
            clean_image = self.undistort_image(clean_image)
            draw_image = clean_image.copy()
            draw_image = self.perform_tag_detection(clean_image, draw_image)

            #if self.is_start:
            #    print("Driving straight: ", self.fixed_maneuvers[self.parking_tag][0])
            #    self.drive_straight(self.fixed_maneuvers[self.parking_tag][0])
            #    self.pause(0.5)
            #    self.rotate(math.pi/2 * 0.45, math.pi * 5 * self.fixed_maneuvers[self.parking_tag][1])
            #    self.pause(0.5)
            #    self.is_start = False

            if self.is_start:
                self.drive_arc(self.arcs[self.parking_tag][0], self.arcs[self.parking_tag][0])
                self.pause(0.5)
                self.is_start = False

            v, omega = pid_controller_v_omega(self.ToI_error, parking_pid, rate_int, False)
            self.set_velocities(v, omega)
            rospy.loginfo(f"Error : {self.ToI_error}, velocity: {v}, omega: {omega}")
            
            '''
            v, omega = pid_controller_v_omega(self.alignment_error, parking_pid, rate_int, False)
            self.set_velocities(-v, omega)
            '''

            if self.ToI_area > 40000:
                print("Area threshold reached: ", self.ToI_area)
                self.set_velocities(0, 0)
                col1 = 0
                col2 = 50
                col3 = 100
                col4 = 150
                col5 = 200

                light_show_s_time = rospy.Time.now()
                rate = rospy.Rate(30)
                while not rospy.is_shutdown():
                    light_show_c_time = rospy.Time.now()
                    if (light_show_c_time - light_show_s_time).to_sec() >= 5:
                        print("time elapsed. Stopping")
                        break
                    
                    col1 += 5 
                    if col1 > 255:
                        col1 = 0
                    col2 += 5
                    if col2 > 255:
                        col2 = 0
                    col3 += 5
                    if col3 > 255:
                        col3 = 0
                    col4 += 5
                    if col4 > 255:
                        col4 = 0
                    col5 += 5
                    if col5 > 255:
                        col5 = 0

                    lights_list = [ColorRGBA(r=col1, g=col2, b=col3, a=255),
                                   ColorRGBA(r=col4, g=col5, b=col1, a=255),
                                   ColorRGBA(r=col2, g=col3, b=col4, a=255),
                                   ColorRGBA(r=col5, g=col1, b=col2, a=255),
                                   ColorRGBA(r=col3, g=col4, b=col5, a=255)]
                    
                    self.led_command.publish(LEDPattern(rgb_vals=lights_list))
                    rate.sleep()
                rate = rospy.Rate(10)
                rate.sleep()

                break

    def on_shutdown(self):
        # on shutdown,
        rospy.loginfo(f"[PARKING.PY] Terminate")
        self.set_velocities(0, 0)


if __name__ == '__main__':
    node = Parking(node_name='part_four')
    #rospy.sleep(2)
    #node.parking()
    rospy.spin()
