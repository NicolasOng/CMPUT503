#!/usr/bin/env python3

# potentially useful for question - 1.1 - 1.4 and 2.1

# import required libraries
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
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

        # camera matrix and distortion coefficients from intrinsic.yaml file
        self.cam_matrix = np.array([[319.2461317458548, 0.0, 307.91668484581703], [0.0, 317.75077109798957, 255.6638447529814], [0.0, 0.0, 1.0]])
        self.dist_coeff = np.array([-0.25706255601943445, 0.045805679651939275, -0.0003584336283982042, -0.0005756902051068707, 0.0])
        
        # color detection parameters in HSV format
        self.red_lower = np.array([136, 87, 111], np.uint8) 
        self.red_upper = np.array([180, 255, 255], np.uint8) 

        self.green_lower = np.array([34, 52, 72], np.uint8) 
        self.green_upper = np.array([82, 255, 255], np.uint8) 

        self.blue_lower = np.array([94, 80, 2], np.uint8) 
        self.blue_upper = np.array([120, 255, 255], np.uint8) 

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
                
                cv2.putText(cv2_img, self.color_to_str[color], (x, y), 
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
        self.draw_contour(Color.YELLOW, cv2_img)
        self.draw_contour(Color.WHITE, cv2_img)
        self.draw_contour(Color.RED, cv2_img)
        self.draw_contour(Color.BLUE, cv2_img)
        self.draw_contour(Color.GREEN, cv2_img)
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

    # add other functions as needed

if __name__ == '__main__':
    node = LaneDetectionNode(node_name='lane_detection_node')
    rospy.spin()