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

        #top half crop
        self.h_crop = 0

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

        # topic to publish color coordinates
        self.color_coords_topic = rospy.Publisher(f"/{self.vehicle_name}/color_coords", String, queue_size=1)

        # degree for best fit lines
        self.degree = 1

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

        # point detection thresholds
        self.point_threshold = 400
        self.side_threshold = 250
        self.left_threshold = self.ground_w / 2 - self.side_threshold
        self.right_threshold = self.ground_w / 2 + self.side_threshold

        # color to BGR dictionary
        self.color_to_bgr = {
            Color.RED : (0, 0, 255),
            Color.BLUE: (255, 0, 0),
            Color.GREEN: (0, 255, 0),
            Color.WHITE: (255, 255, 255),
            Color.YELLOW: (0, 255, 255),
            Color.BLACK: (0, 0, 0),
        }

        # target line for MAE calculation
        self.target_line = [0.0753, self.ground_w / 2]
        self.target_line_left = [0.0753, self.ground_w / 2 - 150]
        self.target_line_right = [0.0753, self.ground_w / 2 + 150]
        self.target_line = [0.0753, self.ground_w / 2]
        if self.degree == 2:
            self.target_line = [0, 0.0753, self.ground_w / 2]
            self.target_line_left = [0.0753, self.ground_w / 2 - 150]
            self.target_line_right = [0, 0.0753, self.ground_w / 2 + 150]

        # topic to publish MAEs
        self.mae_topic = rospy.Publisher(f"/{self.vehicle_name}/maes", String, queue_size=1)

        # topic to publish projected and unprojected image
        self.projected_image_topic = rospy.Publisher(f"/{self.vehicle_name}/projected_image", Image, queue_size=1)
        self.unprojected_image_topic = rospy.Publisher(f"/{self.vehicle_name}/unprojected_image", Image, queue_size=1)

        # toggle for drawing to the camera
        self.draw_bbs = False
        self.draw_lanes = False

        # if the bot puts the yellow line on the left or right
        self.yellow_on_left = True

        # the range for calculating the mean errors
        self.error_lower_threshold = 50
        self.error_upper_threshold = 100

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
    
    def project_image_to_ground(self, image):
        # Apply perspective warp
        return cv2.warpPerspective(image, self.homography_to_ground, (self.ground_w, self.ground_h), flags=cv2.INTER_CUBIC)

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
    
    def rotate_image(self, image):
        '''
        this function rotates the image 90 degrees clockwise
        '''
        image = cv2.transpose(image)
        image = cv2.flip(image, flipCode=1)
        return image
    
    def rotate_image_back(self, image):
        image = self.rotate_image(image)
        image = self.rotate_image(image)
        image = self.rotate_image(image)
        return image
    
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

    def get_color_mask_pixel_list(self, color: Color, cv2_img):
        '''
        this function returns a list of all the pixels in the image that are within the color bounds
        the list is an ndarray of shape (n, 2) with n being the number of pixels
        '''
        # Get binary mask
        mask = self.get_color_mask(color, cv2_img)  # Shape (h, w), values 0 or 255

        # Get coordinates where mask is 255 (note that this is flipped, since this is used with a rotated image)
        y_coords, x_coords = np.where(mask == 255)

        # Convert to (n, 2) shape in (x, y) order
        points = np.column_stack((x_coords, y_coords))

        return points  # Returns an (n, 2) array of (x, y) coordinates
    
    def draw_points(self, points, color, cv2_img):
        for point in points:
            x, y = point
            cv2.circle(cv2_img, (int(x), int(y)), radius=2, color=self.color_to_bgr[color], thickness=-1)
    
    def get_best_fit_line(self, points, degree=1):
        x = points[:, 0].flatten()
        y = points[:, 1].flatten()

        coeffs = np.polyfit(x, y, degree)

        return coeffs
    
    def get_best_fit_line_full(self, color, image, div_coeffs=None, above=True):
        '''
        this function gets and draws the best fit line for the given color.
        also filters out points that are farther in the distance from the camera
        '''
        # rotate the image 90 degrees clockwise
        image = self.rotate_image(image)

        start_time_temp = rospy.Time.now()

        # get the color mask pixels
        points = self.get_color_mask_pixel_list(color, image)
        # filter the pixels for ones below a certain threshold
        points = points[points[:, 0] < self.point_threshold]
        points = points[points[:, 1] > self.left_threshold]
        points = points[points[:, 1] < self.right_threshold]

        end_time_temp = rospy.Time.now()
        duration = (end_time_temp - start_time_temp).to_sec()  # Convert to seconds
        #rospy.loginfo(f"\tColor Mask and filtering: {duration:.6f} seconds")

        start_time_temp = rospy.Time.now()

        # can also use the yellow line as a filter for the white line,
        # so we are only looking at one side of the yellow line
        if div_coeffs is not None:
            # get the yellow line
            x_values = points[:, 0]
            y_poly = np.polyval(div_coeffs, x_values)
            # filter for points on either side of the line
            if above:
                points = points[points[:, 1] >= y_poly]
            else:
                points = points[points[:, 1] <= y_poly]
        
        end_time_temp = rospy.Time.now()
        duration = (end_time_temp - start_time_temp).to_sec()  # Convert to seconds
        #rospy.loginfo(f"\tMore filtering: {duration:.6f} seconds")

        start_time_temp = rospy.Time.now()

        # draw the filtered points
        if self.draw_lanes:
            self.draw_points(points, color, image)
        
        end_time_temp = rospy.Time.now()
        duration = (end_time_temp - start_time_temp).to_sec()  # Convert to seconds
        #rospy.loginfo(f"\tDrawing: {duration:.6f} seconds")

        start_time_temp = rospy.Time.now()

        # get the best fit line
        coeffs = None
        if points.size != 0:
            coeffs = self.get_best_fit_line(points, degree=self.degree)
        
        end_time_temp = rospy.Time.now()
        duration = (end_time_temp - start_time_temp).to_sec()  # Convert to seconds
        #rospy.loginfo(f"\Best Fit Line: {duration:.6f} seconds")

        # rotate the image back
        image = self.rotate_image_back(image)
        return image, coeffs
    
    def get_mae(self, coeff_target, coeff_measured):
        '''
        this function calculates the mean squared error between two lines
        '''
        # get x-values for the range to calculate the error (rotated)
        x_values = np.linspace(self.error_lower_threshold, self.error_upper_threshold, 100)
        # get y-values for both lines
        y_target = np.polyval(coeff_target, x_values)
        y_measured = np.polyval(coeff_measured, x_values)
        # Compute Mean Squared Error (MSE)
        #mse = np.mean((y_measured - y_target) ** 2)
        # compute Mean Absolute Error (MAE)
        # compute Mean Error (ME)
        mae = np.mean(y_measured - y_target)
        return mae
    
    def plot_errors(self, coeff_target, coeff_measured, image):
        '''
        this function plots the error between the target line and the measured line
        '''
        if not self.draw_lanes: return image
        image = self.rotate_image(image)
        # get x-values for the range where the error was calculated
        x_values = np.linspace(self.error_lower_threshold, self.error_upper_threshold, 100)
        # get y-values for both lines
        y_target = np.polyval(coeff_target, x_values)
        y_measured = np.polyval(coeff_measured, x_values)
        # plot the errors
        for x, yt, ym in zip(x_values, y_target, y_measured):
            x, yt, ym = int(x), int(yt), int(ym)
            cv2.line(image, (x, yt), (x, ym), color=self.color_to_bgr[Color.RED], thickness=1)
        image = self.rotate_image_back(image)
        return image
    
    def plot_best_fit_line(self, coeffs, image, color):
        if not self.draw_lanes: return image
        # Generate x and y values for plotting in image
        x_fit = np.linspace(0, self.ground_w, 1000)
        y_fit = np.polyval(coeffs, x_fit)

        # Convert (x, y) into integer pixel coordinates
        curve_points = np.column_stack((x_fit, y_fit)).astype(np.int32)

        # Draw the curve on the OpenCV image
        cv2.polylines(image, [curve_points], isClosed=False, color=self.color_to_bgr[Color.BLACK], thickness=6)
        cv2.polylines(image, [curve_points], isClosed=False, color=self.color_to_bgr[color], thickness=2)

    def plot_best_fit_line_full(self, coeffs, image, color):
        '''
        this function plots the best fit line in the rotated image
        '''
        if not self.draw_lanes: return image
        if coeffs is None or len(coeffs) == 0: return image
        image = self.rotate_image(image)
        self.plot_best_fit_line(coeffs, image, color)
        image = self.rotate_image_back(image)
        return image

    def draw_polygon(self, image, points, color):
        '''
        assumes (n, 2) shaped input
        '''
        # Reshape points to be compatible with polylines function
        points = points.reshape((-1, 1, 2))
        cv2.polylines(image, [points], isClosed=True, color=self.color_to_bgr[color], thickness=2)
    
    def draw_projected_bounding_box(self, image, points, center, coords, color):
        '''
        this function draws the projected bounding box and the ground x, y coordinates
        '''
        if not self.draw_bbs: return
        if points is None or points.size == 0: return
        # draw the bounding box
        points = points.astype(np.int32).reshape((-1, 1, 2))
        self.draw_polygon(image, points, color)
        # draw the center
        cv2.circle(image, (int(center[0]), int(center[1])), radius=2, color=self.color_to_bgr[color], thickness=-1)
        # draw the x, y coordinates
        cv2.putText(image, f"({coords[0]:.2f}, {coords[1]:.2f})", (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_to_bgr[color])

    def draw_MAE_values(self, image, yellow_mae, white_mae):
        '''
        this function draws the MAE values on the image
        '''
        if not self.draw_lanes: return
        if yellow_mae is not None: yellow_mae = round(yellow_mae, 2)
        if white_mae is not None: white_mae = round(white_mae, 2)
        cv2.putText(image, f"Yellow ME: {yellow_mae}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_to_bgr[Color.YELLOW])
        cv2.putText(image, f"White ME: {white_mae}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_to_bgr[Color.BLUE])

    def project_image_from_ground(self, image):
        homography_inv = np.linalg.inv(self.homography_to_ground)
        return cv2.warpPerspective(image, homography_inv, (640, 480), flags=cv2.INTER_CUBIC)
    
    def draw_bounding_box(self, image, bb, center, coords, color):
        '''
        this function draws the bounding box and the ground x, y coordinates
        '''
        if not self.draw_bbs: return
        if bb is None: return
        # draw the bounding box
        x, y, w, h = bb
        cv2.rectangle(image, (x, y), (x + w, y + h), self.color_to_bgr[color], 2) 
        # draw the center
        cv2.circle(image, (int(center[0]), int(center[1])), radius=2, color=self.color_to_bgr[color], thickness=-1)
        # draw the x, y coordinates
        cv2.putText(image, f"({coords[0]:.2f}, {coords[1]:.2f})", (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.color_to_bgr[color])
    
    def perform_camera_detection(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.camera_image is None: continue
            start_time = rospy.Time.now()
            # create a copy of the camera image
            image = self.camera_image.copy()
            # undistort camera image
            image = self.undistort_image(image)
            image = image[int(self.cam_h * self.h_crop):int(self.cam_h), int(0):int(self.cam_w)]

            start_time_temp = rospy.Time.now()

            # get the nearest bounding boxes for red, blue, and green
            red_bb = self.get_nearest_bounding_box(Color.RED, image)
            blue_bb = self.get_nearest_bounding_box(Color.BLUE, image)
            green_bb = self.get_nearest_bounding_box(Color.GREEN, image)
            # get the bounding box centers
            default_center = (-2, -2, 0, 0)
            red_center, blue_center, green_center = default_center, default_center, default_center
            if red_bb is not None:
                red_center = (red_bb[0] + red_bb[2] / 2, red_bb[1] + red_bb[3] / 2)
            if blue_bb is not None:
                blue_center = (blue_bb[0] + blue_bb[2] / 2, blue_bb[1] + blue_bb[3] / 2)
            if green_bb is not None:
                green_center = (green_bb[0] + green_bb[2] / 2, green_bb[1] + green_bb[3] / 2)
            
            end_time_temp = rospy.Time.now()
            duration = (end_time_temp - start_time_temp).to_sec()  # Convert to seconds
            #rospy.loginfo(f"BB calculations duration: {duration:.6f} seconds")

            start_time_temp = rospy.Time.now()

            # project the image to the ground
            image = self.project_image_to_ground(image)

            end_time_temp = rospy.Time.now()
            duration = (end_time_temp - start_time_temp).to_sec()  # Convert to seconds
            #rospy.loginfo(f"ground projection duration: {duration:.6f} seconds")

            start_time_temp = rospy.Time.now()

            # project the centers (and bounding boxes) to the ground
            red_center_p = self.project_point_to_ground(red_center)
            blue_center_p = self.project_point_to_ground(blue_center)
            green_center_p = self.project_point_to_ground(green_center)
            red_bb_p = self.project_bounding_box_to_ground(red_bb)
            blue_bb_p = self.project_bounding_box_to_ground(blue_bb)
            green_bb_p = self.project_bounding_box_to_ground(green_bb)
            # get the x, y coordinates of the projected centers on the ground
            # in mm, relative to the robot's center
            # flip the y-axis so that positive y is forward
            red_coords = (red_center_p[0] - self.robot_x, -(red_center_p[1] - self.robot_y))
            blue_coords = (blue_center_p[0] - self.robot_x, -(blue_center_p[1] - self.robot_y))
            green_coords = (green_center_p[0] - self.robot_x, -(green_center_p[1] - self.robot_y))
            # publish the color detection results
            color_coords = {
                "red": red_coords,
                "blue": blue_coords,
                "green": green_coords
            }
            json_coords = json.dumps(color_coords)
            self.color_coords_topic.publish(json_coords)

            end_time_temp = rospy.Time.now()
            duration = (end_time_temp - start_time_temp).to_sec()  # Convert to seconds
            #rospy.loginfo(f"bb projections and topic duration: {duration:.6f} seconds")

            start_time_temp = rospy.Time.now()

            # perform yellow line detection - get the coefficients
            # also draws the points used
            image, yellow_line = self.get_best_fit_line_full(Color.YELLOW, image)
            # perform white line detection - get the coefficients
            # also draws the points used
            image, white_line_left = self.get_best_fit_line_full(Color.WHITE, image, div_coeffs=self.target_line, above=True)
            image, white_line_right = self.get_best_fit_line_full(Color.WHITE, image, div_coeffs=self.target_line, above=True)

            end_time_temp = rospy.Time.now()
            duration = (end_time_temp - start_time_temp).to_sec()  # Convert to seconds
            #rospy.loginfo(f"calculating BF lines: {duration:.6f} seconds")

            start_time_temp = rospy.Time.now()

            # choose the "canonical" white line, based on where the yellow line toggle.
            # also choose the target lines
            if self.yellow_on_left:
                white_line = white_line_right
                yellow_target_line = self.target_line_left
                white_target_line = self.target_line_right
            else:
                white_line = white_line_left
                yellow_target_line = self.target_line_right
                white_target_line = self.target_line_left
            # get the mid-lane line coefficients
            mid_lane_line = None
            if white_line is not None and white_line.size > 0 and yellow_line is not None and yellow_line.size > 0:
                mid_lane_line = (np.array(yellow_line) + np.array(white_line)) / 2
            # get the errors between the white and yellow lines
            yellow_mae, white_mae = None, None
            if yellow_line is not None and yellow_line.size > 0:
                # calculate the MAE between the yellow line and the target line
                yellow_mae = self.get_mae(yellow_target_line, yellow_line)
            if white_line is not None and white_line.size > 0:
                # calculate MAE between the mid-lane line and the target line
                white_mae = self.get_mae(white_target_line, white_line)
            # publish the MAEs
            maes = {
                "yellow": yellow_mae,
                "white": white_mae,
            }
            json_maes = json.dumps(maes)
            self.mae_topic.publish(json_maes)

            end_time_temp = rospy.Time.now()
            duration = (end_time_temp - start_time_temp).to_sec()  # Convert to seconds
            #rospy.loginfo(f"various line calculations: {duration:.6f} seconds")

            start_time_temp = rospy.Time.now()

            # draw the error lines (yellow and white)
            if yellow_line is not None and yellow_line.size > 0:
                image = self.plot_errors(yellow_target_line, yellow_line, image)
            if white_line is not None and white_line.size > 0:
                image = self.plot_errors(white_target_line, white_line, image)
            # draw the yellow, white, mid-lane (blue), and target (green) lines
            image = self.plot_best_fit_line_full(yellow_line, image, Color.YELLOW)
            image = self.plot_best_fit_line_full(white_line_left, image, Color.WHITE)
            image = self.plot_best_fit_line_full(white_line_right, image, Color.WHITE)
            image = self.plot_best_fit_line_full(mid_lane_line, image, Color.BLUE)
            image = self.plot_best_fit_line_full(self.target_line, image, Color.GREEN)
            image = self.plot_best_fit_line_full(self.target_line_left, image, Color.GREEN)
            image = self.plot_best_fit_line_full(self.target_line_right, image, Color.GREEN)
            # make a copy of the image - save this for un-projection later.
            projected_image = image.copy()
            # draw the projected color bounding boxes and their calculated ground x, y coordinates
            self.draw_projected_bounding_box(image, red_bb_p, red_center_p, red_coords, Color.RED)
            self.draw_projected_bounding_box(image, blue_bb_p, blue_center_p, blue_coords, Color.BLUE)
            self.draw_projected_bounding_box(image, green_bb_p, green_center_p, green_coords, Color.GREEN)
            # crop and draw the 2 error values
            image = image[int(self.ground_h * 0.3):int(self.ground_h), int(0):int(self.ground_w)]
            self.draw_MAE_values(image, yellow_mae, white_mae)
            # crop and publish the projected image
            self.projected_image_topic.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))
            # un-project the image copy to the camera frame
            image = projected_image
            image = self.project_image_from_ground(image)
            # draw the color bounding boxes and their calculated ground x, y coordinates
            self.draw_bounding_box(image, red_bb, red_center, red_coords, Color.RED)
            self.draw_bounding_box(image, green_bb, green_center, green_coords, Color.GREEN)
            self.draw_bounding_box(image, blue_bb, blue_center, blue_coords, Color.BLUE)
            # draw the 2 error values
            self.draw_MAE_values(image, yellow_mae, white_mae)
            # publish the un-projected image
            self.unprojected_image_topic.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))

            end_time_temp = rospy.Time.now()
            duration = (end_time_temp - start_time_temp).to_sec()  # Convert to seconds
            #rospy.loginfo(f"plotting and publishing: {duration:.6f} seconds")

            rate.sleep()
            end_time = rospy.Time.now()
            duration = (end_time - start_time).to_sec()  # Convert to seconds
            rospy.loginfo(f"Loop duration: {duration:.6f} seconds")
            rospy.loginfo(f"---")

    def on_shutdown(self):
        # on shutdown
        pass

if __name__ == '__main__':
    node = CameraDetectionNode(node_name='camera_detection_node')
    rospy.sleep(2)
    node.perform_camera_detection()
    rospy.spin()
