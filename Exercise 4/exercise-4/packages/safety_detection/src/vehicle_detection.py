#!/usr/bin/env python3

# potentially useful for part 2 of exercise 4

# import required libraries
import rospy
from duckietown.dtros import DTROS, NodeType
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
import numpy as np

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
        self.other_bot_info_pub = rospy.Publisher(f"/{self.vehicle_name}/other_bot_info", String, queue_size=1)
        self.lane_error_topic = rospy.Publisher(f"/{self.vehicle_name}/lane_error", String, queue_size=1)

        # robot position in the projected ground plane,
        # below the center of the image by some distance (mm)
        self.ground_w, self.ground_h = 1250, 1250
        self.robot_x, self.robot_y = self.ground_w / 2, self.ground_h + 100
        self.cam_w, self.cam_h = 640, 480


        # define other variables as needed
        self.img = None

        self.last_stamp = rospy.Time.now()
        self.script_dir = os.path.dirname(os.path.abspath(__file__))

        homography_file = np.load(os.path.join(self.script_dir, "homography.npz"))

        # Access arrays by their keys
        self.homography_to_ground = homography_file["homography_to_ground"]
        self.cam_matrix = homography_file["cam_matrix"]
        self.dist_coeff = homography_file["dist_coeff"]

        self.callback_freq = 10  # hz
        self.publish_duration = rospy.Duration.from_sec(1.0 / self.callback_freq)  # in seconds

        # color to BGR dictionary
        self.color_to_bgr = {
            Color.RED : (0, 0, 255),
            Color.BLUE: (255, 0, 0),
            Color.GREEN: (0, 255, 0),
            Color.WHITE: (255, 255, 255),
            Color.YELLOW: (0, 255, 255),
            Color.BLACK: (0, 0, 0),
        }

        # flags
        self.stop_flag = False
        return

    def fill_blob_detector_params(self):
        self.blob_detector_params.filterByArea = True
        self.blob_detector_params.minArea = 5  # pixels
        self.blob_detector_params.maxArea = 1000  # pixels
                # Filter by circularity
        self.blob_detector_params.filterByCircularity = True
        self.blob_detector_params.minCircularity = 0.7

        # Filter by convexity
        self.blob_detector_params.filterByConvexity = True
        self.blob_detector_params.minConvexity = 0.8

        # Filter by inertia
        self.blob_detector_params.filterByInertia = True
        self.blob_detector_params.minInertiaRatio = 0.8


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
        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:  # process every 0.1 seconds
            return
        else:
            self.last_stamp = now

        image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        # undistort the image
        undistorted_image = self.undistort_image(image_cv)
        self.img = undistorted_image


        blob_points = self.detect_bot(undistorted_image)
        if blob_points is None: return


        other_bot_coord = self.get_other_bot_coord(blob_points)

        # msg 
        other_bot_msg = {
            "other_bot_coord": other_bot_coord,  # x, y of the other bot relative to the bot
        }
        json_le = json.dumps(str(other_bot_msg))
        self.other_bot_info_pub.publish(json_le)




    """
    call either find_cirle_grid or approximation
    
    Args:
        image_cv: cv2 image
    Returns:
        centers: list of (x, y) tuples of the circle grid centers OR NONE
    """
    def detect_bot(self, image_cv):
        # find the circle grid
        (detection, centers) = self.find_circle_grid(image_cv)

        if centers is None: return None

        if centers is not None and centers.shape[0] == 21:
            # good 
            return centers
        else:
            blob_points = self.simple_blob_detector.detect(image_cv) # get the points
            # remove outliers
            blob_points = self.remove_outliers(blob_points, distance_threshold=200)

            if len(blob_points) == 0: return None
            return blob_points
    """
    
    Args:
        circle_points: list of (x, y) tuples of the circle grid centers
        we removed the outliers from the circle_points
    
    """
    def get_other_bot_coord(self, circle_points):
        assert circle_points is not None

        # get circle grid dim 
        (grid_width, grid_height) = self.approximate_duckiebot_circlegrid_dim(circle_points)

        if len(circle_points) == 21:
            # distance of the centers
            middle_column_center = circle_points[17].squeeze() # 11th center is the middle column center of the 7x3 grid
            # project the middle point to the ground
            proj_middle = self.project_point_to_ground([middle_column_center[0], middle_column_center[1] - 3.5*grid_height]) 
        else:
            assert len(circle_points) > 0
            # get the mean of the points
            mean_point = np.mean(circle_points, axis=0) # dim = 2
            proj_middle = self.project_point_to_ground([mean_point[0], mean_point[1] - 3.5*grid_height])
        return proj_middle  # dim 2

    def find_circle_grid(self, image_cv):
        # undistort the image
        #undistorted_image = self.undistort_image(image_cv)
        # find the circle grid
        (detection, centers) = cv2.findCirclesGrid(
            image_cv,
            patternSize=(7, 3),
            flags=cv2.CALIB_CB_SYMMETRIC_GRID,
            blobDetector=self.simple_blob_detector,
        )
        
        return (detection, centers)

    """
    circle_points is from simple_blob_detector.detect(image_cv), converted to a list of (x, y) tuples

    """
    def approximate_duckiebot_circlegrid_dim(self, circle_points):
        # get the points with maximum and minimum x coordinates
        max_x = max(circle_points, key=lambda x: x[0])   
        min_x = min(circle_points, key=lambda x: x[0])

        max_y = max(circle_points, key=lambda x: x[1])   # max
        min_y = min(circle_points, key=lambda x: x[1])
        # min

        circle_grid_width = max_x[0] - min_x[0]
        circle_grid_height = max_y[1] - min_y[1]

        return (circle_grid_width, circle_grid_height)

    """
    points is output of simple_blob_detector.detect(image_cv)
    returns the points that are within the distance threshold from the mean of the points
    """
    def remove_outliers(self, points, distance_threshold):
        # remove outliers
        points = np.array([p.pt for p in points])
        # get the mean of the points
        good_points = []
        for i in range(len(points)):
            for j in range(len(points)):
                if i == j:
                    continue
                if np.linalg.norm(points[i] - points[j]) < distance_threshold:
                    good_points.append(points[i])
                    break
        return good_points


    # TODO: subscribe to the undistorted image topic if camera_detection node publishes it
    def undistort_image(self, cv2_img):
        h, w = cv2_img.shape[:2]
        # optimal camera matrix lets us see entire camera image (image edges cropped without), but some distortion visible
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.cam_matrix, self.dist_coeff, (w,h), 0, (w,h))

        # undistorted image using calibration parameters
        return cv2.undistort(cv2_img, self.cam_matrix, self.dist_coeff, None)


    def put_text(self, img, text, position):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_color = (0, 255, 0)
        font_thickness = 1
        cv2.putText(img, text, position, font, font_scale, font_color, font_thickness)

    """
    ARCHIVED. moved everything to the image_callback
    """
    def loop(self):
        # add your code here
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # check if the vehicle is detected
            if self.img is not None:
                # find the circle grid
                (detection, centers) = self.find_circle_grid(self.img)
                if detection is None: continue
                if centers is None: continue    

                if centers.shape[0] == 21:
                    print(f'Found the circle grid')
                    # distance of the centers
                    middle_column_center = centers[17].squeeze() # 11th center is the middle column center of the 7x3 grid
                    top_left = centers[0].squeeze()              # top left corner of the circle grid coord of pixel
                    bottom_left = centers[14].squeeze()
                    bottom_right = centers[20].squeeze()
                    circle_grid_height = (top_left[1] - bottom_left[1])  # height of circle grid in pixels
                    circle_grid_width = (bottom_right[0] - bottom_left[0])  # width of circle grid in pixels

                    # project the middle point to the ground
                    proj_middle = self.project_point_to_ground([middle_column_center[0], middle_column_center[1] - 3.5*circle_grid_height]) 
                    
                    error = (self.cam_w /2 - middle_column_center[0])
                    white_line_on_right = True  # TODO: get this from camera_detection node
                    if white_line_on_right:
                        # bot must swerve to the left
                        safe_target = bottom_left[0] - 1.5*circle_grid_width
                    else:
                        # bot must swerve to the right
                        safe_target = bottom_right[0] + 1.5*circle_grid_width

                    # publish this as an error in the lane errors topic
                    other_bot_msg = {
                        "other_bot_coord": proj_middle,  # x, y of the other bot relative to the bot
                        "error": error,    # signed distance between center of the bot and the center circlegrid
                        "safe_target": safe_target  # target x coordinate for the bot to go to to avoid collision
                    }
                    json_le = json.dumps(str(other_bot_msg))
                    self.other_bot_info_pub.publish(json_le)
                    cv2.drawChessboardCorners(self.img, (7, 3), centers, detection)
                    self.put_text(self.img, str(proj_middle), (10, 30))
                    self.put_text(self.img, f'error: {error}', (10, 60))
                    self.draw_vertical_line(self.img, safe_target, Color.RED)
                
                self.circle_img_pub.publish(self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8"))
            rate.sleep()

    def draw_vertical_line(self, image, x, color):
        '''
        draws a vertical line at the given x-coordinate
        '''
        x = int(x)
        cv2.line(image, (x, 0), (x, image.shape[0]), color=self.color_to_bgr[color], thickness=1)

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
    

if __name__ == '__main__':
    # create the node
    node = VehicleDetection(node_name='april_tag_detector')
    #node.loop()
    rospy.spin()

"""
Circle grid indices
(0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6)  # First row
(1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6)  # Second row
(2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6)  # Third row

"""
