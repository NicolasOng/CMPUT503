#!/usr/bin/env python3
import json
import math
import os

import cv2
from cv_bridge import CvBridge
import dt_apriltags
import numpy as np

import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Pose2DStamped, WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped, LEDPattern
from std_msgs.msg import ColorRGBA, String

from camera_detection import CameraDetectionNode
from Color import Color
from pid_controller import simple_pid, pid_controller_v_omega
from safety_detection.srv import SetString, SetStringResponse

class Parking(DTROS):
    def __init__(self, node_name):
        super(Parking, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # camera subscriber
        self.camera_image = None
        self.bridge = CvBridge()
        self.camera_sub = rospy.Subscriber(f"/{self.vehicle_name}/camera_node/image/compressed", CompressedImage, self.camera_callback)
        self.camera_sub

        # odometry topic
        self.ctheta = 0
        self.cpos = 0
        self.lane_error_topic = rospy.Subscriber(f"/{self.vehicle_name}/odometry", String, self.odometry_callback)
        
        # tag detection
        self.draw_atag_toggle = True
        self.is_ToI = False
        self.ToI_area = 0
        self.parking_tag = 228
        self.at_detector = dt_apriltags.Detector()
        self.tag_image_sub = rospy.Publisher(f"/{self.vehicle_name}/tag_image", Image, queue_size=1)

        # camera matrix and distortion coefficients from intrinsic.yaml file
        self.cam_matrix = np.array([
            [319.2461317458548, 0.0, 307.91668484581703],
            [0.0, 317.75077109798957, 255.6638447529814],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeff = np.array([-0.25706255601943445, 0.045805679651939275, -0.0003584336283982042, -0.0005756902051068707, 0.0])


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
        rospy.loginfo(f'linear: {linear}, omega: {rotational}')


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


    def rotate(self, radians, speed):
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


    # Draws a bounding box and ID on an ApriltTag 
    def draw_atag_features(self, image, points, id, center, colour=(255, 100, 255)):
        h, w = image.shape[:2]
        tag_offset_error = str(center[0] - w//2)
        img = cv2.polylines(image, [points], True, colour, 5)
        img = cv2.putText(image, tag_offset_error, self.tag_center, cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,100,255), 1)
        img = cv2.line(image, (w//2, h//2), center, (255,100,255), 2)
        #img = cv2.putText(image, id, (center[0], center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.75, colour, 2)
        return img
    

    def perform_tag_detection(self, clean_image, draw_image):
        # Convert image to grayscale
        image_grey = cv2.cvtColor(clean_image, cv2.COLOR_BGR2GRAY)
        image_grey = cv2.GaussianBlur(image_grey, (5,5), 0)

        # ApriltTag detector
        results = self.at_detector.detect(image_grey)

        ToI_index = -1
        self.is_ToI = False
        self.ToI_area = 0

        if len(results) == 0:
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
            
            # WEBCAM IMAGE COORDINATES: TOP LEFT is 0,0
            # DUCKIEBOT CAMERA COORDINATES ARE REVERSED: BOTTOM LEFT is 0,0
            tl = ToI.corners[0].astype(int)
            br = ToI.corners[2].astype(int)
            ToI_area = (br[0] - tl[0]) * (br[1] - tl[1])
            self.ToI_area = ToI_area
            
            if self.draw_atag_toggle:
                draw_image = self.draw_atag_features(draw_image, ToI_corners, ToI_id, ToI_center)

        self.tag_image_sub.publish(self.bridge.cv2_to_imgmsg(draw_image, encoding="bgr8"))

        return draw_image


    def parking(self):
        rate_int = 10
        rate = rospy.Rate(rate_int)
        while not rospy.is_shutdown():

            clean_image = self.camera_image.copy()
            # undistort camera image
            clean_image = self.undistort_image(clean_image)
            h, w = clean_image.shape[:2]

            draw_image = clean_image.copy()
            draw_image = self.perform_tag_detection(clean_image, draw_image)

            if self.ToI_area > 115000:
                print("Area threshold reached")

            # Center line of image
            #draw_image = cv2.line(draw_image, (w//2, 0), (w//2, h), (0,255,0), 2)
            #draw_image = cv2.circle(draw_image, (w//2, h//2), 3, (0,255,0), 3)



            '''
            start_time = rospy.Time.now()

            # do the lane following
            v, omega = pid_controller_v_omega(self.lane_error, simple_pid, rate_int, False)
            self.set_velocities(v, omega)

            rate.sleep()
            # update the cooldowns
            end_time = rospy.Time.now()
            dt = (end_time - start_time).to_sec()
            rospy.loginfo(f"Loop duration: {dt:.6f} seconds")
            rospy.loginfo(f"---")
            '''


    def on_shutdown(self):
        # on shutdown,
        rospy.loginfo(f"[PARKING.PY] Terminate")
        self.set_velocities(0, 0)


if __name__ == '__main__':
    node = Parking(node_name='parking')
    rospy.sleep(2)
    node.parking()
    rospy.spin()
