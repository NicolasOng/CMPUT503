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
import dt_apriltags

NO_TAG_ID = "-1"

class CameraDetectionNode(DTROS):
    def __init__(self, node_name):
        super(CameraDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # camera subscriber
        self.camera_image = None
        self.bridge = CvBridge()
        self.camera_sub = rospy.Subscriber(f"/{self.vehicle_name}/camera_node/image/compressed", CompressedImage, self.camera_callback)
        self.camera_sub

        # Publishers
        self.color_coords_topic = rospy.Publisher(f"/{self.vehicle_name}/color_coords", String, queue_size=1)
        self.lane_error_topic = rospy.Publisher(f"/{self.vehicle_name}/lane_error", String, queue_size=1)
        self.camera_detection_image_topic = rospy.Publisher(f"/{self.vehicle_name}/camera_detection_image", Image, queue_size=1)
        self.yellow_cropped_image_topic = rospy.Publisher(f"/{self.vehicle_name}/camera_yellow_cropped", Image, queue_size=1)
        self.duckiebot_pub = rospy.Publisher(f"/{self.vehicle_name}/duckiebot_area", String, queue_size=1)

        # AprilTag Publishers
        self.tag_id_pub = rospy.Publisher(f"/{self.vehicle_name}/tag_id", String, queue_size=1)
        self.tag_list_pub = rospy.Publisher(f"/{self.vehicle_name}/tag_list", String, queue_size=1)
        self.tag_image = rospy.Publisher(f"/{self.vehicle_name}/tag_image", Image, queue_size=1)
        self.duckies_pub = rospy.Publisher(f"/{self.vehicle_name}/duckies_info", String, queue_size=1)

        self.id_sub = rospy.Subscriber(f"/{self.vehicle_name}/tag_id", String, self.id_callback)
        self.white_line_right_topic = rospy.Subscriber(f"/{self.vehicle_name}/white_line_right", String, self.white_line_loc_cb)  # True/False if white line is on right

        self.tag_id = None

        # AprilTag detector engine. Expensive to create/destroy, so create a single instance for class, call detect as needed
        self.at_detector = dt_apriltags.Detector()

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
        dst_pts_translation = np.array([(self.ground_w / 2) - 24, self.ground_h - 255], dtype=np.float32)
        src_pts = np.array([[284, 285], [443, 285], [273, 380], [584, 380]], dtype=np.float32)
        dst_pts = np.array([[0, 0], [186, 0], [0, 186], [186, 186]], dtype=np.float32)
        dst_pts = dst_pts + dst_pts_translation
        self.homography_to_ground, _ = cv2.findHomography(src_pts, dst_pts)

        # robot position in the projected ground plane,
        # below the center of the image by some distance (mm)
        self.robot_x, self.robot_y = self.ground_w / 2, self.ground_h + 100

        # Color Detection Stuff
        # color detection parameters in HSV format
        # Define range
        hr = 40
        sr = 50
        vr = 50
        #red_hsv = np.array([0, 144, 255])  # rGB: 255, 111, 111
        #self.red_lower = np.array([max(0, red_hsv[0] - hr), max(0, red_hsv[1] - sr), max(0, red_hsv[2] - vr)])
        #self.red_upper = np.array([min(179, red_hsv[0] + hr), min(255, red_hsv[1] + sr), min(255, red_hsv[2] + vr)])
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

        self.orange_lower = np.array([30/2, 50*2.55, 30*2.55], np.uint8)
        self.orange_higher = np.array([36/2, 100*2.55, 100*2.55], np.uint8)
        # color bounds
        self.color_bounds = {
            Color.RED: (self.red_lower, self.red_upper),
            Color.BLUE: (self.blue_lower, self.blue_upper),
            Color.GREEN: (self.green_lower, self.green_upper),
            Color.YELLOW: (self.yellow_lower, self.yellow_higher),
            Color.WHITE: (self.white_lower, self.white_higher),
            Color.ORANGE: (self.orange_lower, self.orange_higher),
        }

        # color to BGR dictionary
        self.color_to_bgr = {
            Color.RED : (0, 0, 255),
            Color.BLUE: (255, 0, 0),
            Color.GREEN: (0, 255, 0),
            Color.WHITE: (255, 255, 255),
            Color.YELLOW: (0, 255, 255),
            Color.BLACK: (0, 0, 0),
            Color.ORANGE: (0, 165, 255)
        }
        
        # Draw Toggles
        self.draw_lane_detection = False
        self.draw_bounding_boxes = False
        self.draw_atag_toggle = False
        self.draw_duckies = False
        self.draw_duckiebot = False

        # if the bot puts the white line on the right or left
        self.white_on_right = True

        # offset for simple lane detection
        self.simple_offset = 100

        # threshold for ground/horizon
        self.horizon = self.cam_h * 0.6

        # masks for gorund bounding box detection
        self.polygon_points = np.array([
            [60 * self.cam_w // 100, self.cam_h/2],   # Top-right
            [40 * self.cam_w // 100, self.cam_h/2],       # Top-left
            [5 * self.cam_w // 100, self.cam_h],       # bottom-left
            [95 * self.cam_w // 100, self.cam_h],   # bottom-right
        ], np.int32)
        self.lane_mask = np.zeros((self.cam_h, self.cam_w), dtype=np.uint8)
        cv2.fillPoly(self.lane_mask, [self.polygon_points], 255)

        self.polygon_points_white = np.array([
            [55 * self.cam_w // 100, self.cam_h/2],   # Top-right
            [45 * self.cam_w // 100, self.cam_h/2],       # Top-left
            [20 * self.cam_w // 100, self.cam_h],       # bottom-left
            [80 * self.cam_w // 100, self.cam_h],   # bottom-right
        ], np.int32)
        self.lane_mask_white = np.zeros((self.cam_h, self.cam_w), dtype=np.uint8)
        cv2.fillPoly(self.lane_mask_white, [self.polygon_points_white], 255)

        self.polygon_points_bottom = np.array([
            [0, 0],   # Top-left
            [self.cam_w, 0],       # Top-right
            [self.cam_w, 80 * self.cam_h // 100],       # bottom-right
            [0, 80 * self.cam_h // 100],   # bottom-left
        ], np.int32)
        self.bottom_mask = np.zeros((self.cam_h, self.cam_w), dtype=np.uint8)
        cv2.fillPoly(self.bottom_mask, [self.polygon_points_bottom], 255)
    
    def white_line_loc_cb(self, msg):
        m_json = msg.data
        self.white_on_right = bool(json.loads(m_json)["white_line_right"])

    def id_callback(self, msg):
        self.tag_id = int(msg.data)

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
    
    def get_ground_bounding_boxes(self, color, cv2_img):
        # get the color mask
        color_mask = self.get_color_mask(color, cv2_img)
        # only look at colors in the lane:
        color_mask = cv2.bitwise_and(color_mask, self.lane_mask)
        if color == Color.WHITE:
            color_mask = cv2.bitwise_and(color_mask, self.lane_mask_white)
        # get the color contours
        contours, hierarchy = self.get_contours(color_mask)
        # get all the bounding boxes with a large area, and on the ground.
        bbs = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if (area > 300): 
                x, y, w, h = cv2.boundingRect(contour)
                contour_bb = (x, y, w, h)
                contour_center = (x + w / 2, y + h / 2)
                if contour_center[1] > self.horizon:
                    bbs.append({"bb": contour_bb, "center": contour_center})
        return bbs
    
    def project_point_to_ground(self, point):
        '''
        point is a tuple of (x, y) coordinates
        the point is relative to the bot.
        '''
        point = np.array([point], dtype=np.float32)
        new_point = cv2.perspectiveTransform(point.reshape(-1, 1, 2), self.homography_to_ground)
        new_point = new_point.ravel()
        new_point = (new_point[0] - self.robot_x, -(new_point[1] - self.robot_y))
        return new_point
    
    def project_bounding_box_to_ground(self, bounding_box):
        '''
        bounding_box is a tuple of (x, y, w, h) coordinates
        output is a list of 4 points in the ground plane.
        These points are relative to the robot.
        '''
        if not bounding_box: return None
        x, y, w, h = bounding_box
        points = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], dtype=np.float32)
        new_points = cv2.perspectiveTransform(points.reshape(-1, 1, 2), self.homography_to_ground)
        new_points = new_points.reshape(-1, 2)
        transformed_coords = np.column_stack([
            new_points[:, 0] - self.robot_x,
            -(new_points[:, 1] - self.robot_y)
        ])
        return transformed_coords
    
    def project_bounding_boxes_to_ground(self, bbs):
        pbbs = []
        for bb in bbs:
            pbb = self.project_bounding_box_to_ground(bb["bb"])
            pc = self.project_point_to_ground(bb["center"])
            data = {"bb": pbb, "center": pc}
            serializable_data = {
                'bb': data['bb'].tolist(),  # Convert the np.array to a Python list
                'center': (float(data['center'][0]), float(data['center'][1]))  # Convert np.float32 to float
            }
            pbbs.append(serializable_data)
        return pbbs
    
    def draw_bounding_boxes_to_image(self, image, bbs, projected_bbs, color):
        for i in range(len(bbs)):
            bb = bbs[i]
            pbb = projected_bbs[i]
            self.draw_bounding_box(image, bb["bb"], bb["center"], pbb["center"], color)

    def perform_ground_color_detection(self, clean_image, draw_image):
        if self.camera_image is None: return draw_image
        # get all the bounding boxes above a certain area for each color (white, red, blue)
        # also in the bottom half of the image
        red_bbs = self.get_ground_bounding_boxes(Color.RED, clean_image)
        white_bbs = self.get_ground_bounding_boxes(Color.WHITE, clean_image)
        blue_bbs = self.get_ground_bounding_boxes(Color.BLUE, clean_image)
        # project the bounding boxes (and their centers) to the ground
        red_bbs_p = self.project_bounding_boxes_to_ground(red_bbs)
        white_bbs_p = self.project_bounding_boxes_to_ground(white_bbs)
        blue_bbs_p = self.project_bounding_boxes_to_ground(blue_bbs)
        # publish the info to the topic
        color_coords = {
            "red": red_bbs_p,
            "white": white_bbs_p,
            "blue": blue_bbs_p
        }
        json_coords = json.dumps(color_coords)
        self.color_coords_topic.publish(json_coords)
        # draw the bounding boxes and their centers, with their center ground coordinates
        if self.draw_bounding_boxes:
            cv2.line(draw_image, tuple(self.polygon_points[0]), tuple(self.polygon_points[3]), (0, 255, 0), 2)  # Left line
            cv2.line(draw_image, tuple(self.polygon_points[1]), tuple(self.polygon_points[2]), (0, 255, 0), 2)  # Right line
            cv2.line(draw_image, tuple(self.polygon_points_white[0]), tuple(self.polygon_points_white[3]), (0, 255, 0), 2)  # Left line
            cv2.line(draw_image, tuple(self.polygon_points_white[1]), tuple(self.polygon_points_white[2]), (0, 255, 0), 2)  # Right line
            self.draw_bounding_boxes_to_image(draw_image, red_bbs, red_bbs_p, Color.RED)
            self.draw_bounding_boxes_to_image(draw_image, white_bbs, white_bbs_p, Color.WHITE)
            self.draw_bounding_boxes_to_image(draw_image, blue_bbs, blue_bbs_p, Color.BLUE)
        return draw_image
    
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
        # crop image to a strip around the bottom
        image = clean_image[int(self.cam_h * 0.7):int(self.cam_h * 0.9), int(0):int(self.cam_w)]
        ## crop the left or right half off
        x_offset = 0
        if self.white_on_right:
            x_offset = int(self.cam_w / 2)
            image = image[:, int(self.cam_w / 2):int(self.cam_w)]
        else:
            image = image[:, int(0):int(self.cam_w / 2)]
        # WHITE LINE ----- do color detection for the white line, get the biggest white blob
        white_bb = self.get_largest_bounding_box(Color.WHITE, image)
        white_center = None
        if white_bb is not None:
            white_center = (white_bb[0] + white_bb[2] / 2, white_bb[1] + white_bb[3] / 2)
        # get its distance from the left side of the image, plus some offset
        white_error = None
        if white_center is not None:
            if self.white_on_right:
                # negative error - bot should turn left.
                white_error = white_center[0] - (self.cam_w / 2 - self.simple_offset)
            else:
                white_error = white_center[0] - (0 + self.simple_offset)

        # ----------- YELLOW lINE DETECTION (assume yellow is always on the left) -----------
        image_yellow_crop = clean_image[int(self.cam_h * 0.7):int(self.cam_h * 0.9), int(0):int(self.cam_w)]
        image_yellow_crop = image_yellow_crop[:, int(0):int(self.cam_w / 2)]  # bottom left of the image
        yellow_bb = self.get_largest_bounding_box(Color.YELLOW, image_yellow_crop)
        yellow_center = None
        if yellow_bb is not None:
            yellow_center = (yellow_bb[0] + yellow_bb[2] / 2, yellow_bb[1] + yellow_bb[3] / 2)

        yellow_error = None
        if yellow_center is not None:
            yellow_error = yellow_center[0] - (0 + self.simple_offset)
        # ---------------------------------------------------------
        #final_error = white_error
        #if yellow_error is not None:
        #    if white_error is None:
        #        final_error = yellow_error
        #    else:
        #        final_error = (white_error + yellow_error) / 2
        # todo remove
        #final_error = yellow_error

        lane_errors = {
            "yellow_lane_error": yellow_error,
            "white_lane_error": white_error,
        }
        #rospy.loginfo(f"Lane error: {error}")
        json_le = json.dumps(lane_errors)
        self.lane_error_topic.publish(json_le)
        # draw image for visualization
        if self.draw_lane_detection:
            line_1 = int(self.simple_offset + x_offset)
            cv2.line(draw_image, (line_1, int(self.cam_h * 0.7)), (line_1, int(self.cam_h * 0.9)), color=self.color_to_bgr[Color.BLUE], thickness=1)
            line_2 = int(self.simple_offset)
            cv2.line(draw_image, (line_2, int(self.cam_h * 0.7)), (line_2, int(self.cam_h * 0.9)), color=self.color_to_bgr[Color.BLUE], thickness=1)
            #if white_bb is not None:
            #    draw_white_bb = list(white_bb)
            #    draw_white_bb[0] += int(x_offset)
            #    draw_white_bb[1] += int(self.cam_h * 0.7)
            #    draw_white_center = list(white_center)
            #    draw_white_center[0] += int(x_offset)
            #    draw_white_center[1] += int(self.cam_h * 0.7)
            #    self.draw_bounding_box(draw_image, draw_white_bb, draw_white_center, white_center, Color.BLUE)
            if yellow_bb is not None:
                x_offset = 0 
                draw_yellow_bb = list(yellow_bb)
                draw_yellow_bb[0] += int(x_offset)
                draw_yellow_bb[1] += int(self.cam_h * 0.7)
                draw_yellow_center = list(yellow_center)
                draw_yellow_center[0] += int(x_offset)
                draw_yellow_center[1] += int(self.cam_h * 0.7)
                self.draw_bounding_box(draw_image, draw_yellow_bb, draw_yellow_center, yellow_center, Color.YELLOW)

            final_error = yellow_error if yellow_error is not None else white_error
            self.draw_lane_error_value(draw_image, final_error)
            #self.yellow_cropped_image_topic.publish(self.bridge.cv2_to_imgmsg(image_yellow_crop, encoding="bgr8"))
        return draw_image
    
    # Draws a bounding box and ID on an ApriltTag 
    def draw_atag_features(self, image, tl, br, id, center, colour=(255, 100, 255)):
        cv2.rectangle(image, (br[0], br[1]), (tl[0], tl[1]), colour, 5) 
        cv2.putText(image, id, (center[0], center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.75, colour, 2)

    def perform_tag_detection(self, clean_image, draw_image):
        tags_list = []
        if self.camera_image is None: 
            return clean_image

        # Convert image to grayscale
        image_grey = cv2.cvtColor(clean_image, cv2.COLOR_BGR2GRAY)
        image_grey = cv2.GaussianBlur(image_grey, (5,5), 0)

        # ApriltTag detector
        results = self.at_detector.detect(image_grey)

        largest_tag_index = 0
        largest_tag_area = 0
        id = -1

        if len(results) == 0:
            self.tag_image.publish(self.bridge.cv2_to_imgmsg(clean_image, encoding="bgr8"))
            self.tag_id_pub.publish(NO_TAG_ID)
            return draw_image
        # If multiple tags detected, find most prominent tag (largest by area)
        #elif len(results) > 1:
        else:
            for idx, r in enumerate(results):
                tl = r.corners[0].astype(int)
                br = r.corners[2].astype(int)
                area = (tl[1] - br[1]) * (br[0] - tl[0])
                if area > largest_tag_area:
                    largest_tag_index = idx
                    largest_tag_area = area

        # Order tags in list based on area (most prominent first), set list to empty if none detected
                tags_list.append({"id" : r.tag_id, 
                                  "center" : r.center, 
                                  "corners" : r.corners,
                                  "area" : area})
                
            tags_list = sorted(tags_list, key=lambda dict: dict["area"], reverse=True)

        largest_tag = results[largest_tag_index]
        top_left = largest_tag.corners[0].astype(int)
        bottom_right = largest_tag.corners[2].astype(int)
        center = largest_tag.center.astype(int)
        id = str(largest_tag.tag_id)
        
        if self.draw_atag_toggle: 
            self.draw_atag_features(draw_image, top_left, bottom_right, id, center)

        #self.tag_list.publish(json.dumps(tags_list))
        self.tag_id_pub.publish(id)  
        self.tag_image.publish(self.bridge.cv2_to_imgmsg(clean_image, encoding="bgr8"))
                  
        return draw_image
    
    def perform_duckie_detection(self, clean_image, draw_image):
        mask = self.get_color_mask(Color.ORANGE, clean_image)  # (h, w)
        duckie_exist = False
        min_point = None   

        big_contours = []
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:
                x, y, w, h = cv2.boundingRect(contour)
                big_contours.append((x, y, w, h))
                cv2.rectangle(draw_image, (x, y), (x + w, y + h), (0, 165, 255), 2) 

        duckie_exist = len(big_contours) > 0

        # find the (x, y) coordinates of the white pixel in mask with minimum y value
        if duckie_exist:
            y, x = np.where(mask)
            min_y_index = np.argmin(y)
            min_y = y[min_y_index]
            min_x = x[min_y_index]
            min_point = (min_x, min_y)
            #rospy.loginfo(f"detect duckie at {min_point}")

            min_point = self.project_point_to_ground(min_point)

        msg = {
            "duckie_exist": duckie_exist,
            "min_point": min_point
        }
        self.duckies_pub.publish(json.dumps(msg))

        if self.draw_duckies:
            for contour in big_contours:
                x, y, w, h = contour
                cv2.rectangle(draw_image, (x, y), (x + w, y + h), (0, 165, 255), 2)
        
        return draw_image 
    
    def convert_hsv(self, hue, saturation, value):
        '''
        from custom:
        Hue: [0, 355]
        Saturation: [0, 100]
        Value: [0, 100]
        to opencv:
        Hue: [0,179]
        Saturation: [0,255]
        Value: [0,255]
        '''
        # Convert hue from [0, 355] to [0, 179]
        ocv_hue = int((hue / 355) * 179)

        # Convert saturation and value from [0, 100] to [0, 255]
        ocv_saturation = int((saturation / 100) * 255)
        ocv_value = int((value / 100) * 255)

        return [ocv_hue, ocv_saturation, ocv_value]

    def perform_duckie_bot_detection(self, clean_image, draw_image):
        lower = np.array(self.convert_hsv(140, 40, 20), np.uint8)
        upper = np.array(self.convert_hsv(240, 100, 100), np.uint8)
        hsv_frame = cv2.cvtColor(clean_image, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)

        # Create color mask
        color_mask = cv2.inRange(hsv_frame, lower, upper)
        color_mask = cv2.dilate(color_mask, kernel)

        # mask out bottom part of the image
        # only look at colors in the lane:
        color_mask = cv2.bitwise_and(color_mask, self.bottom_mask)

        # Get the number of white pixels (value = 255)
        mask_area = cv2.countNonZero(color_mask)
        msg = {
            "duckiebot_mask_area": mask_area,
        }
        self.duckiebot_pub.publish(json.dumps(msg))
        #rospy.loginfo(f"MASK SIZE: {mask_area}")
        
        if self.draw_duckiebot:
            big_contours = []
            contours, _ = cv2.findContours(color_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 1000:
                    x, y, w, h = cv2.boundingRect(contour)
                    big_contours.append((x, y, w, h))
                    cv2.rectangle(draw_image, (x, y), (x + w, y + h), (153, 135, 46), 2)
                    cv2.putText(draw_image, f"{mask_area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (153, 135, 46), 2)
        
        return draw_image 
        
    def perform_camera_detection(self):
        rate = rospy.Rate(10)
        i = 0
        image_dir = "camera_images"
        os.makedirs(image_dir, exist_ok=True)
        while not rospy.is_shutdown():
            start_time = rospy.Time.now()
            if self.camera_image is None: continue
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
            draw_image = self.perform_ground_color_detection(clean_image.copy(), draw_image)
            # perform apriltags detection
            #draw_image = self.perform_tag_detection(clean_image.copy(), draw_image)

            # perform duckie detection
            draw_image = self.perform_duckie_detection(clean_image.copy(), draw_image)

            draw_image = self.perform_duckie_bot_detection(clean_image.copy(), draw_image)
                
            # publish the image
            self.camera_detection_image_topic.publish(self.bridge.cv2_to_imgmsg(draw_image, encoding="bgr8"))

            # save the image:
            if False:
                img_filename = os.path.join(image_dir, f"frame_{i:04d}.png")
                i += 1
                cv2.imwrite(img_filename, draw_image)
            # end the loop iteration
            rate.sleep()
            end_time = rospy.Time.now()
            duration = (end_time - start_time).to_sec()
            #rospy.loginfo(f"Loop duration: {duration:.6f} seconds")
            #rospy.loginfo(f"---")


    def on_shutdown(self):
        # on shutdown
        print("terminate")

if __name__ == '__main__':
    node = CameraDetectionNode(node_name='camera_detection_node')
    rospy.sleep(2)
    node.perform_camera_detection()
    rospy.spin()
