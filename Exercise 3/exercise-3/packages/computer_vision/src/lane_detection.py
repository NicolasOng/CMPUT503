#!/usr/bin/env python3

# potentially useful for question - 1.1 - 1.4 and 2.1

# import required libraries
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Pose2DStamped, WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped, LEDPattern
from Color import Color
import cv2
from cv_bridge import CvBridge

class LaneDetectionNode(DTROS):
    def __init__(self, node_name):
        super(LaneDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # add your code here
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # subscribers
        self.camera_sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        # publishers
        self.undistorted_pub = rospy.Publisher(f"{self._vehicle_name}/undistorted", Image, queue_size=10)
        self.blur_pub = rospy.Publisher(f"{self._vehicle_name}/blur", Image, queue_size=10)
        self.resize_pub = rospy.Publisher(f"{self._vehicle_name}/resize", Image, queue_size=10)
        self.car_cmd = rospy.Publisher(f"/{self._vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)

        # camera matrix and distortion coefficients from intrinsic.yaml file
        self.cam_matrix = np.array([[319.2461317458548, 0.0, 307.91668484581703], [0.0, 317.75077109798957, 255.6638447529814], [0.0, 0.0, 1.0]])
        self.dist_coeff = np.array([-0.25706255601943445, 0.045805679651939275, -0.0003584336283982042, -0.0005756902051068707, 0.0])
        
        # from extrinsic.yaml file
        self.homography = np.array([[-0.00013668875104344582, 0.0005924050290243054, -0.5993724660928124], [-0.0022949507610645035, -1.5331615246117395e-05, 0.726763100835842], [0.00027302496335237673, 0.017296161892938217, -2.946528752705874]])

        # color detection parameters in HSV format
        self.red_lower = np.array([136, 87, 111], np.uint8) 
        self.red_upper = np.array([180, 255, 255], np.uint8) 

        self.green_lower = np.array([34, 52, 72], np.uint8) 
        self.green_upper = np.array([82, 255, 255], np.uint8) 

        self.blue_lower = np.array([110, 80, 120], np.uint8) 
        self.blue_upper = np.array([130, 255, 255], np.uint8) 

        """
        yellow H: [21, 33], S: [100, 255], V = [153, 255]  # H range 0-170. S range 0-255. V range 0-100

        white H: [0, 170], S: [0, 15], V: [255, 255]
        """
        self.yellow_lower = np.array([21, 100, 60*2.55], np.uint8)
        self.yellow_higher = np.array([33, 255, 100*2.55], np.uint8)

        self.white_lower = np.array([0, 0, 200], np.uint8)  # for white. any value of Hue works. just maximum brighteness
        self.white_higher = np.array([170, 25, 255], np.uint8)
        # initialize bridge and subscribe to camera feed
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)

        self.color_to_str = {
            Color.RED : "red",
            Color.BLUE: "blue",
            Color.GREEN: "green",
            Color.WHITE: "white",
            Color.YELLOW: "yellow",
        }

        # opencv channel is bgr instead of rgb
        self.color_to_bgr = {
            Color.RED : (0, 0, 255),
            Color.BLUE: (255, 0, 0),
            Color.GREEN: (0, 255, 0),
            Color.WHITE: (255, 255, 255),
            Color.YELLOW: (0, 255, 255),
        }
        # lane detection publishers

        # LED
        
        # ROI vertices
        
        # define other variables as needed
        self.cam_y, self.cam_x = 480, 640

        # camera image
        self.camera_image = None

        # PID controller variables
        self.kp = 0  # Proportional gain
        self.ki = 0  # Integral gain
        self.kd = 0  # Derivative gain
        self.previous_error = 0
        self.integral = 0
    
    """
    project a point from the camera frame to the ground plane
    
    """
    def project_points_to_ground(self, point):
        x, y = point

        point = np.array([x, y, 1])

        ground_point = np.dot(self.homography, point)
        ground_point /= ground_point[2]  # normalize by z
        
        return ground_point[:2]

    def project_image_to_ground(self, image):
        """
        Applies the homography transformation to the entire image.
        
        :param image: Input cv2 image (numpy array).
        :return: Warped image.
        """
        h, w = image.shape[:2]  # Get the height and width of the image

        # Apply the homography transformation
        warped_image = cv2.warpPerspective(image, self.homography, (w, h))

        return warped_image

    """
    the l2 distance between two points
    """
    def get_distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    """
    get the distance from the center of the image to the color objects
    Args:
        color: color enum {red, blue, green, yellow, white}
        cv2_img: img
    return:
        list of distances from the center of the image to the detected color objects
    """
    def get_distance_from_color(self, color, cv2_img):
        # get the color mask
        color_mask = self.get_color_mask(color, cv2_img)
        # get the color contours
        contours, hierarchy = self.get_contours(color_mask)
        
        # get the distance from the center of the image to the color
        distances = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            contour_center = (x + w / 2, y + h / 2)
            projected_center_contour = self.project_points_to_ground(contour_center)

            distances.append(abs(projected_center_contour[0]))
        return distances
    
    
    def get_pid_controls(self, measured_value, desired_value, dt, reset=False):
        '''
        The method to get PID controls.
        For P/PD, just set ki and/or kd to 0
        use the reset flag when the desired value changes a lot
        need to tune the kp, ki, kd values for different tasks (keep a note of them)
        '''
        if reset:
            self.integral = 0
            self.previous_error = 0
        error = desired_value - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        
        return output
    
    def balance_yellow(self):
        '''
        during this function, the robot should be constantly moving forward.
        the robot will also turn left or right to balance the yellow color on the left and right side of its image feed.
        this is done using color detection and PID controls.
        '''
        linear = 0.5
        rate_hz = 10
        dt = 1 / rate_hz
        rate = rospy.Rate(rate_hz)
        # wait for the camera feed to start
        while self.camera_image is None:
            rate.sleep()
        while not rospy.is_shutdown():
            left_yellow, right_yellow = self.get_yellow_balance(self.camera_image)
            yellow_balance = left_yellow - right_yellow
            rotational = self.get_pid_controls(yellow_balance, 0, dt)
            self.car_cmd.publish(Twist2DStamped(v=linear, omega=rotational))
            rate.sleep()
    
    def undistort_image(self, cv2_img):
        # add your code here
        h,  w = cv2_img.shape[:2]
        # optimal camera matrix lets us see entire camera image (image edges cropped without), but some distortion visible
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.cam_matrix, self.dist_coeff, (w,h), 0, (w,h))


        # undistorted image using calibration parameters
        undistorted_cv2img = cv2.undistort(cv2_img, self.cam_matrix, self.dist_coeff, None)

        #undistorted_cv2img = cv2.cvtColor(undistorted_cv2img, cv2.COLOR_BGR2RGB)
        return undistorted_cv2img

    def preprocess_image(self, **kwargs):
        # add your code here
        pass
    
    
    """
    Return a Binary color mask from the cv2_img. Used to draw contours
    Args:
        color: color enum {red, blue, gree, yellow, white}
        cv2_img: img
    return:
        a binary mask
    """
    def get_color_mask(self, color: Color, cv2_img):
        hsvFrame = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV) 
        kernel = np.ones((5, 5), "uint8") 
        color_mask = None
        if color == Color.RED:
            # For red color 
            color_mask = cv2.inRange(hsvFrame, self.red_lower, self.red_upper)
            color_mask = cv2.dilate(color_mask, kernel) 
            res_color = cv2.bitwise_and(cv2_img, cv2_img, 
                                    mask = color_mask) 
        elif color == Color.BLUE:
            # For red color 
            color_mask = cv2.inRange(hsvFrame, self.blue_lower, self.blue_upper)
            color_mask = cv2.dilate(color_mask, kernel) 
            res_color = cv2.bitwise_and(cv2_img, cv2_img, 
                                    mask = color_mask) 
        elif color == Color.GREEN:
            # For red color 
            color_mask = cv2.inRange(hsvFrame, self.green_lower, self.green_upper)
            color_mask = cv2.dilate(color_mask, kernel) 
            res_color = cv2.bitwise_and(cv2_img, cv2_img, 
                                    mask = color_mask) 
        elif color == Color.YELLOW:
            # For yellow color 
            color_mask = cv2.inRange(hsvFrame, self.yellow_lower, self.yellow_higher)
            color_mask = cv2.dilate(color_mask, kernel) 
            res_color = cv2.bitwise_and(cv2_img, cv2_img, 
                                    mask = color_mask) 
        elif color == Color.WHITE:
            # For white color 
            color_mask = cv2.inRange(hsvFrame, self.white_lower, self.white_higher)
            color_mask = cv2.dilate(color_mask, kernel) 
            res_color = cv2.bitwise_and(cv2_img, cv2_img, 
                                    mask = color_mask) 
        assert color_mask is not None
        return color_mask

    def get_contours(self, color_mask):
        contours, hierarchy = cv2.findContours(color_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
        return contours, hierarchy

    """
    Draw bounding box around the color objects in the img
    Args:
        color: color enum (red, blue green yellow, white)
        cv2_img: img
    return:
        None
    """
    def draw_contour(self, color: Color, cv2_img):
        color_mask = self.get_color_mask(color, cv2_img)
        color_bgr = self.color_to_bgr[color]  # (0-255, 0-255, 0-255) bgr format

        # Creating contour to track red color 
        contours, hierarchy = cv2.findContours(color_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
                cv2_img = cv2.rectangle(cv2_img, (x, y), 
                                        (x + w, y + h), 
                                        color_bgr, 2) 
                
                contour_center = (x + w / 2, y + h / 2)
                projected_center = self.project_points_to_ground(contour_center)

                cv2.putText(cv2_img, self.color_to_str[color] + " x: " + str(abs(projected_center[0])), (x, y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                            color_bgr)     
        return


    """
    Currently just draws contour
    Args:
        cv2_img: img
    return:
        None 
    """
    def detect_lane_color(self, cv2_img):
        # add your code here
        # color space 
        #self.draw_contour(Color.YELLOW, cv2_img)
        #self.draw_contour(Color.WHITE, cv2_img)
        #self.draw_contour(Color.RED, cv2_img)
        self.draw_contour(Color.BLUE, cv2_img)
        #self.draw_contour(Color.GREEN, cv2_img)
        return cv2_img
    
    def get_yellow_balance(self, cv2_img):
        left_yellow = 0
        right_yellow = 0
        mid_x = self.cam_x / 2
        # get the yellow color mask
        color_mask = self.get_color_mask(Color.YELLOW, cv2_img)
        # get the yellow contours (vectors of coordinates in each blob in the mask)
        contours, hierarchy = cv2.findContours(color_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE)
        # for each contour,
        for pic, contour in enumerate(contours):
            # for each coordinate in the contour,
            for coord in contour:
                # get the x and y coordinates
                x, y = coord[0]
                if x < mid_x:
                    left_yellow += 1
                else:
                    right_yellow += 1
        return left_yellow, right_yellow
    
    def display_yellow_balance(self, cv2_img):
        mid_x = self.cam_x / 2
        left_yellow, right_yellow = self.get_yellow_balance(cv2_img)
        cv2.putText(cv2_img, f'{left_yellow}, {right_yellow}', (int(mid_x), self.cam_y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                            self.color_to_bgr[Color.YELLOW])
        return cv2_img
    
    def detect_lane(self, **kwargs):
        # add your code here
        # potentially useful in question 2.1
        pass
    
    """
    Callback for /csc22946/camera_node/image/compressed topic.
    Undistort image and run color detection on it
    
    """
    def callback(self, msg):
        # add your code here
        
        # convert compressed image to CV2
        cv2_image = self._bridge.compressed_imgmsg_to_cv2(msg)
        # undistort image
        undistort_cv2_img = self.undistort_image(cv2_image)
        # preprocess image
        undistort_cv2_img = self.detect_lane_color(undistort_cv2_img)

        # save image for methods not in this callback
        self.camera_image = undistort_cv2_img
        # yellow balance
        #undistort_cv2_img = self.display_yellow_balance(undistort_cv2_img)
        #undistort_cv2_img = self.project_image_to_ground(undistort_cv2_img)




        # display frame
        cv2.imshow(self._window, undistort_cv2_img)
        cv2.waitKey(1)
        # detect lanes - 2.1 
        
        # publish lane detection results
        
        # detect lanes and colors - 1.3
        
        # publish undistorted image
        undistort_cv2_img = cv2.cvtColor(undistort_cv2_img, cv2.COLOR_BGR2RGB)
        msg_undistorted = self._bridge.cv2_to_imgmsg(undistort_cv2_img, encoding="rgb8")
        self.undistorted_pub.publish(msg_undistorted)
        
        # control LEDs based on detected colors

        # anything else you want to add here
        
        pass

    def drive_until_close_to_color(self, color, threshold):
        # add your code here
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.car_cmd.publish(Twist2DStamped(v=0.5, omega=0))
            if self.camera_image is None: 
                continue
            distances = self.get_distance_from_color(color, self.camera_image)
            if len(distances) == 0:
                continue
            min_distance = min(distances)
            print('min_distance', min_distance)
            if min_distance < threshold:
                self.car_cmd.publish(Twist2DStamped(v=0, omega=0))
                break
            rate.sleep()
        pass

    def on_shutdown(self):
        # on shutdown,
        # stop the wheels
        self.car_cmd.publish(Twist2DStamped(v=0, omega=0))

if __name__ == '__main__':
    node = LaneDetectionNode(node_name='lane_detection_node')
    rospy.sleep(2)
    #node.drive_until_close_to_color(Color.BLUE, 0.0)
    rospy.spin()
