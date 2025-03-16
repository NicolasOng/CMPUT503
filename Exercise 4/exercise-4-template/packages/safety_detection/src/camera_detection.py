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

class CameraDetectionNode(DTROS):
    def __init__(self, node_name):
        super(CameraDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # camera subscriber
        self.camera_image = None
        self.bridge = CvBridge()
        self.camera_sub = rospy.Subscriber(f"/{self.vehicle_name}/camera_node/image/compressed", CompressedImage, self.camera_callback)

        # Publishers
        self.color_coords_topic = rospy.Publisher(f"/{self.vehicle_name}/color_coords", String, queue_size=1)
        self.lane_error_topic = rospy.Publisher(f"/{self.vehicle_name}/lane_error", String, queue_size=1)
        self.camera_detection_image_topic = rospy.Publisher(f"/{self.vehicle_name}/camera_detection_image", Image, queue_size=1)

        # camera matrix and distortion coefficients from intrinsic.yaml file
        self.cam_matrix = np.array([
            [319.2461317458548, 0.0, 307.91668484581703],
            [0.0, 317.75077109798957, 255.6638447529814],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeff = np.array([-0.25706255601943445, 0.045805679651939275, -0.0003584336283982042, -0.0005756902051068707, 0.0])
        
        # from extrinsic.yaml file
        self.homography = np.array([
            [-0.00013668875104344582, 0.0005924050290243054, -0.5993724660928124],
            [-0.0022949507610645035, -1.5331615246117395e-05, 0.726763100835842],
            [0.00027302496335237673, 0.017296161892938217, -2.946528752705874]
        ])

        # projection to ground plane homography matrix
        self.cam_w, self.cam_h = 640, 480
        self.ground_w, self.ground_h = 1250, 1250
        src_pts_translation = np.array([0, -(self.cam_h * self.h_crop)], dtype=np.float32)
        dst_pts_translation = np.array([(self.ground_w / 2) - 24, self.ground_h - 255], dtype=np.float32)
        src_pts = np.array([[284, 285], [443, 285], [273, 380], [584, 380]], dtype=np.float32)
        dst_pts = np.array([[0, 0], [186, 0], [0, 186], [186, 186]], dtype=np.float32)
        src_pts = src_pts + src_pts_translation
        dst_pts = dst_pts + dst_pts_translation
        self.homography_to_ground, _ = cv2.findHomography(src_pts, dst_pts)

        # robot position in the projected ground plane,
        # below the center of the image by some distance (mm)
        self.robot_x, self.robot_y = self.ground_w / 2, self.ground_h + 100

        # Color Detection Stuff
        # color detection parameters in HSV format
        self.red_lower = np.array([136, 87, 111], np.uint8)
        self.red_upper = np.array([180, 255, 255], np.uint8)

        self.green_lower = np.array([34, 52, 72], np.uint8)
        self.green_upper = np.array([82, 255, 255], np.uint8)

        self.blue_lower = np.array([110, 80, 120], np.uint8)
        self.blue_upper = np.array([130, 255, 255], np.uint8)

        self.yellow_lower = np.array([21, 100, 60*2.55], np.uint8)
        self.yellow_higher = np.array([33, 255, 100*2.55], np.uint8)

        self.white_lower = np.array([0, 0, 180], np.uint8)
        self.white_higher = np.array([180, 50, 255], np.uint8)

        # color bounds
        self.color_bounds = {
            Color.RED: (self.red_lower, self.red_upper),
            Color.BLUE: (self.blue_lower, self.blue_upper),
            Color.GREEN: (self.green_lower, self.green_upper),
            Color.YELLOW: (self.yellow_lower, self.yellow_higher),
            Color.WHITE: (self.white_lower, self.white_higher),
        }

        # color to BGR dictionary
        self.color_to_bgr = {
            Color.RED : (0, 0, 255),
            Color.BLUE: (255, 0, 0),
            Color.GREEN: (0, 255, 0),
            Color.WHITE: (255, 255, 255),
            Color.YELLOW: (0, 255, 255),
            Color.BLACK: (0, 0, 0),
        }
        
        # Draw Toggles
        self.draw_lane_detection = True

        # if the bot puts the white line on the right or left
        self.white_on_right = True

        # offset for simple lane detection
        self.simple_offset = 100

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
    
    def get_color_mask(self, color: Color, cv2_img):
        '''
        the color mask gets all the pixels in the image that are within the color bounds
        the color mask is an ndarray of shape (h, w) with values 0 or 255.
        0 means the pixel is not in the color bounds, 255 means it is
        '''
        hsv_frame = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)

        # Get the lower and upper bounds for the given color
        lower, upper = self.color_bounds.get(color, (None, None))
        assert lower is not None and upper is not None, f"Invalid color: {color}"

        # Create color mask
        color_mask = cv2.inRange(hsv_frame, lower, upper)
        color_mask = cv2.dilate(color_mask, kernel)

        return color_mask
    
    def get_contours(self, color_mask):
        '''
        using the color mask, we can get the contours of the color
        the contours are the edges of the color, defined by a list of points
        contours is a tuple of ndarrays of shape (n, 1, 2)
        '''
        contours, hierarchy = cv2.findContours(color_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
        return contours, hierarchy
    
    def get_distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    def get_nearest_bounding_box(self, color, cv2_img):
        # get the color mask
        color_mask = self.get_color_mask(color, cv2_img)
        # get the color contours
        contours, hierarchy = self.get_contours(color_mask)
        # get the nearest bounding box (to the bottom middle of the image)
        nearest_bb = None
        nearest_distance = float('inf')
        for contour in contours:
            area = cv2.contourArea(contour)
            if (area > 300): 
                x, y, w, h = cv2.boundingRect(contour)
                contour_center = (x + w / 2, y + h / 2)
                distance = self.get_distance((self.cam_w / 2, self.cam_h), contour_center)
                if distance < nearest_distance:
                    nearest_distance = distance
                    nearest_bb = (x, y, w, h)
        return nearest_bb
    
    def project_point_to_ground(self, point):
        '''
        point is a tuple of (x, y) coordinates
        '''
        point = np.array([point], dtype=np.float32)
        new_point = cv2.perspectiveTransform(point.reshape(-1, 1, 2), self.homography_to_ground)
        return new_point.ravel()
    
    def project_bounding_box_to_ground(self, bounding_box):
        '''
        bounding_box is a tuple of (x, y, w, h) coordinates
        output is a list of 4 points in the ground plane
        '''
        if not bounding_box: return None
        x, y, w, h = bounding_box
        points = np.array([[x, y], [x + w, y], [x, y + h], [x + w, y + h]], dtype=np.float32)
        new_points = cv2.perspectiveTransform(points.reshape(-1, 1, 2), self.homography_to_ground)
        return new_points.reshape(-1, 2)
    
    def perform_ground_color_detection(self):
        if self.camera_image is None: return
        # create a copy of the camera image
        image = self.camera_image.copy()
        # undistort camera image
        image = self.undistort_image(image)
        image = image[int(self.cam_h * self.h_crop):int(self.cam_h), int(0):int(self.cam_w)]

        # get the nearest bounding boxes for red, blue, and green
        red_bb = self.get_nearest_bounding_box(Color.RED, image)
        # get the bounding box centers
        default_center = (-2, -2, 0, 0)
        red_center = default_center
        if red_bb is not None:
            red_center = (red_bb[0] + red_bb[2] / 2, red_bb[1] + red_bb[3] / 2)
        
        # project the centers (and bounding boxes) to the ground
        red_center_p = self.project_point_to_ground(red_center)
        red_bb_p = self.project_bounding_box_to_ground(red_bb)
        # get the x, y coordinates of the projected centers on the ground
        # in mm, relative to the robot's center
        # flip the y-axis so that positive y is forward
        red_coords = (red_center_p[0] - self.robot_x, -(red_center_p[1] - self.robot_y))
        # publish the color detection results
        color_coords = {
            "red": red_coords
        }
        json_coords = json.dumps(color_coords)
        self.color_coords_topic.publish(json_coords)
        # draw the color bounding boxes and their calculated ground x, y coordinates
        self.draw_bounding_box(image, red_bb, red_center, red_coords, Color.RED)

    def get_largest_bounding_box(self, color, cv2_img):
        # get the color mask
        color_mask = self.get_color_mask(color, cv2_img)
        # get the color contours
        contours, hierarchy = self.get_contours(color_mask)
        # get the largest bounding box
        largest_bb = None
        largest_area = -float('inf')
        for contour in contours:
            area = cv2.contourArea(contour)
            if (area > largest_area):
                x, y, w, h = cv2.boundingRect(contour)
                largest_area = area
                largest_bb = (x, y, w, h)
        return largest_bb
    
    def draw_vertical_line(self, image, x, color):
        '''
        draws a vertical line at the given x-coordinate
        '''
        x = int(x)
        cv2.line(image, (x, 0), (x, image.shape[0]), color=self.color_to_bgr[color], thickness=1)
    
    def draw_bounding_box(self, image, bb, center, coords, color):
        '''
        this function draws the bounding box and the ground x, y coordinates
        '''
        if bb is None: return
        # draw the bounding box
        x, y, w, h = bb
        cv2.rectangle(image, (x, y), (x + w, y + h), self.color_to_bgr[color], 2) 
        # draw the center
        cv2.circle(image, (int(center[0]), int(center[1])), radius=2, color=self.color_to_bgr[color], thickness=-1)
        # draw the x, y coordinates
        cv2.putText(image, f"({coords[0]:.2f}, {coords[1]:.2f})", (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_to_bgr[color])
    
    def draw_lane_error_value(self, image, lane_error):
        '''
        this function draws the lane error values on the image
        '''
        if lane_error is not None: lane_error = round(lane_error, 2)
        cv2.putText(image, f"Lane Error: {lane_error}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_to_bgr[Color.BLUE])
    
    def perform_simple_lane_detection(self, clean_image, draw_image):
        if self.camera_image is None: return draw_image
        # create a copy of the clean image
        image = clean_image.copy()
        # crop image to a strip around the bottom
        image = image[int(self.cam_h * 0.7):int(self.cam_h * 0.9), int(0):int(self.cam_w)]
        # crop the left or right half off
        if self.white_on_right:
            image = image[:, int(self.cam_w / 2):int(self.cam_w)]
        else:
            image = image[:, int(0):int(self.cam_w / 2)]
        # do color detection for the white line, get the biggest white blob
        white_bb = self.get_largest_bounding_box(Color.WHITE, image)
        white_center = None
        if white_bb is not None:
            white_center = (white_bb[0] + white_bb[2] / 2, white_bb[1] + white_bb[3] / 2)
        # get its distance from the left side of the image, plus some offset
        error = None
        if white_center is not None:
            if self.white_on_right:
                # negative error - bot should turn left.
                error = white_center[0] - (self.cam_w / 2 - self.simple_offset)
            else:
                error = white_center[0] - (0 + self.simple_offset)
        # publish this as an error in the lane errors topic
        lane_errors = {
            "lane_error": error
        }
        json_le = json.dumps(lane_errors)
        self.lane_error_topic.publish(json_le)
        # draw image for visualization
        if self.draw_lane_detection:
            self.draw_vertical_line(image, 0 + self.simple_offset, Color.BLUE)
            self.draw_vertical_line(image, self.cam_w / 2 - self.simple_offset, Color.BLUE)
            self.draw_bounding_box(image, white_bb, white_center, white_center, Color.BLUE)
            self.draw_lane_error_value(image, error)
        return draw_image
        
    def perform_camera_detection(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            start_time = rospy.Time.now()
            # create a copy of the camera image
            image = self.camera_image.copy()
            # undistort camera image
            image = self.undistort_image(image)
            # get the clean and draw image
            clean_image = image.copy()
            draw_image = image
            # perform lane detection
            draw_image = self.perform_simple_lane_detection(clean_image.copy(), draw_image)
            # peform colored tape detection
            self.perform_ground_color_detection()
            # perform apriltags detection

            # perform other detection (duckiebot from behind, pedestrians, etc)

            # publish the image
            self.camera_detection_image_topic.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))
            # end the loop iteration
            rate.sleep()
            end_time = rospy.Time.now()
            duration = (end_time - start_time).to_sec()
            rospy.loginfo(f"Loop duration: {duration:.6f} seconds")
            rospy.loginfo(f"---")


    def on_shutdown(self):
        # on shutdown
        pass

if __name__ == '__main__':
    node = CameraDetectionNode(node_name='camera_detection_node')
    rospy.sleep(2)
    #node.perform_camera_detection()
    node.perform_simple_camera_detection()
    rospy.spin()
