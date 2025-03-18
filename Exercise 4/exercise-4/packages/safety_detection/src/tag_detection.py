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

class AprilTagDetectionNode(DTROS):
    def __init__(self, node_name):
        super(AprilTagDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # camera subscriber
        self.camera_image = None
        self.bridge = CvBridge()
        self.camera_sub = rospy.Subscriber(f"/{self.vehicle_name}/camera_node/image/compressed", CompressedImage, self.camera_callback)

        # Publishers
        self.tag_id = rospy.Publisher(f"/{self.vehicle_name}/tag_ids", int, queue_size=1)
        #self.tag_center = rospy.Publisher(f"/{self.vehicle_name}/tag_center", String, queue_size=1)
        #self.tag_corners = rospy.Publisher(f"/{self.vehicle_name}/tag_corners", String, queue_size=1)
        self_tag_list = rospy.Publisher(f"/{self.vehicle_name}/tag_list", String, queue_size=1)
        self.tag_image = rospy.Publisher(f"/{self.vehicle_name}/tag_image", Image, queue_size=1)

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
    
    # Draws a bounding box and ID on an ApriltTag 
    def draw_atag_features(self, image, tl, br, id, center, colour=(255, 100, 255)):
        cv2.rectangle(image, (br[0], br[1]), (tl[0], tl[1]), colour, 5) 
        cv2.putText(image, id, (center[0], center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.75, colour, 2)

    def detect_tags(self, clean_image):
        if self.camera_image is None: 
            return clean_image
        
        image = clean_image.copy()

        # Convert image to grayscale
        image_grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # ApriltTag detector
        results = self.at_detector.detect(image_grey)

        print("Number of detected tags: ", len(results))

        tags_list = []

        largest_tag_index = 0
        largest_tag_area = 0

        if len(results) == 0:
            self.tag_image.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))
            return
        
        # If multiple tags detected, find most prominent tag (largest by area)
        if len(results) > 1:
            for idx, r in enumerate(results):
                tl = r.corners[0].astype(int)
                br = r.corners[2].astype(int)
                area = (tl[1] - br[1]) * (br[0] - tl[0])
                if area > largest_tag_area:
                    largest_tag_index = idx
                    largest_tag_area = area

                tags_list.append({"id" : r.tag_id, "center" : r.center, "corners" : r.corners})

        largest_tag = results[largest_tag_index]

        top_left = largest_tag.corners[0].astype(int)
        bottom_right = largest_tag.corners[2].astype(int)
        center = largest_tag.center.astype(int)
        id = str(largest_tag.tag_id)
        
        self.draw_atag_features(image, top_left, bottom_right, id, center)

        # Publish data on all detected tags
        self.tag_list.publish(json.dumps(tags_list))
        # Publish image of most prominently detected tag
        self.tag_image.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))
        # Publish ID of most prominently detected tag
        self.tag_id.publish(id)
        return 
    
    def perform_tag_detection(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # create a copy of the camera image
            image = self.camera_image.copy()
            # undistort camera image
            image = self.undistort_image(image)

            # get the clean and draw image
            clean_image = image.copy()
            draw_image = image

            draw_image = self.detect_tags(clean_image.copy())

    def on_shutdown(self):
        # on shutdown
        pass

if __name__ == '__main__':
    node = AprilTagDetectionNode(node_name='tag_detection_node')
    rospy.sleep(2)
    #node.perform_camera_detection()
    node.perform_tag_detection()
    rospy.spin()
