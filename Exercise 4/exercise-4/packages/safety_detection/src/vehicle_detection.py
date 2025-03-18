#!/usr/bin/env python3

# potentially useful for part 2 of exercise 4

# import required libraries
import rospy
from duckietown.dtros import DTROS, NodeType
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Pose2DStamped, WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped, LEDPattern
from std_msgs.msg import ColorRGBA, String
from Color import Color
import cv2
from cv_bridge import CvBridge

class VehicleDetection(DTROS):

    def __init__(self, node_name):
        super(VehicleDetection, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)

        # add your code here
        self.vehicle_name = os.environ['VEHICLE_NAME']

        # camera subscriber
        self.camera_image = None
        self.bridge = CvBridge()
        self.camera_sub = rospy.Subscriber(f"/{self.vehicle_name}/camera_node/image/compressed", CompressedImage, self.image_callback)

        self.blob_detector_params = cv2.SimpleBlobDetector_Params()  # https://stackoverflow.com/questions/8076889/how-to-use-opencv-simpleblobdetector
        self.fill_blob_detector_params()
        self.simple_blob_detector = cv2.SimpleBlobDetector_create(self.blob_detector_params)
        self.circle_img_pub = rospy.Publisher(f"/{self.vehicle_name}/circle_img", Image, queue_size=1)
        # call navigation control node

        # subscribe to camera feed

        # define other variables as needed
        self.detection = None
        self.centers = None
        self.img = None

        self.last_stamp = rospy.Time.now()
    
    def fill_blob_detector_params(self):
        self.blob_detector_params.filterByArea = True
        self.blob_detector_params.minArea = 10  # pixels
        self.blob_detector_params.maxArea = 1000  # pixels
                # Filter by circularity
        self.blob_detector_params.filterByCircularity = True
        self.blob_detector_params.minCircularity = 0.7

        # Filter by convexity
        self.blob_detector_params.filterByConvexity = True
        self.blob_detector_params.minConvexity = 0.8

        # Filter by inertia
        self.blob_detector_params.filterByInertia = True
        self.blob_detector_params.minInertiaRatio = 0.5


    def detect_bot(self, **kwargs):
        pass

    def manuver_around_bot(self, **kwargs):
        pass

    def image_callback(self, image_msg):
        """
        Callback for processing a image which potentially contains a back pattern. Processes the image only if
        sufficient time has passed since processing the previous image (relative to the chosen processing frequency).

        The pattern detection is performed using OpenCV's `findCirclesGrid <https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?highlight=solvepnp#findcirclesgrid>`_ function.

        Args:
            image_msg (:obj:`sensor_msgs.msg.CompressedImage`): Input image

        """

        image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")

        (detection, centers) = cv2.findCirclesGrid(
            image_cv,
            patternSize=(7, 3),
            flags=cv2.CALIB_CB_SYMMETRIC_GRID,
            blobDetector=self.simple_blob_detector,
        )
        self.detection = detection
        self.centers = centers
        self.img = image_cv

    def put_text(self, img, text, position):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_color = (0, 255, 0)
        font_thickness = 1
        cv2.putText(img, text, position, font, font_scale, font_color, font_thickness)


    def loop(self):
        # add your code here
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # check if the vehicle is detected
            print("Detection: ", self.detection)
            if self.detection is not None and self.centers is not None and self.img is not None:
                # call the navigation
                # If the grid is found, draw the circles
                cv2.drawChessboardCorners(self.img, (3, 7), self.centers, self.detection)
                # write center.shape onto img 

                 # Write the shape of `centers` onto the image
                text = f"Centers shape: {self.centers.shape}"
                position = (10, 30)  # Position of the text (bottom-left corner)
                self.put_text(self.img, text, position)

                # Add the text to the image
                # Display the result
                print(f'center size {self.centers.shape}')
            
            if self.img is not None:
                self.circle_img_pub.publish(self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8"))
            rate.sleep()
if __name__ == '__main__':
    # create the node
    node = VehicleDetection(node_name='april_tag_detector')
    node.loop()
    rospy.spin()
