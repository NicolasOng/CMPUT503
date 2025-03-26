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
from safety_detection.srv import SetString, SetStringResponse

from pid_controller import pid_controller_v_omega, simple_pid

class VehicleDetection(DTROS):

    def __init__(self, node_name):
        super(VehicleDetection, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)

        # add your code here
        self.vehicle_name = os.environ['VEHICLE_NAME']
        self.last_stamp = rospy.Time.now()
        self.callback_freq = 100 # hz
        self.publish_duration = rospy.Duration.from_sec(1.0 / self.callback_freq)  # in seconds

        self.script_dir = os.path.dirname(os.path.abspath(__file__))

        homography_file = np.load(os.path.join(self.script_dir, "homography.npz"))

        # Access arrays by their keys
        self.homography_to_ground = homography_file["homography_to_ground"]
        self.cam_matrix = homography_file["cam_matrix"]
        self.dist_coeff = homography_file["dist_coeff"]
        # from extrinsic calibration
        self.homography = np.array([[-0.00013668875104344582, 0.0005924050290243054, -0.5993724660928124], [-0.0022949507610645035, -1.5331615246117395e-05, 0.726763100835842], [0.00027302496335237673, 0.017296161892938217, -2.946528752705874]])
        # camera subscriber
        self.camera_image = None
        self.bridge = CvBridge()

        self.blob_detector_params = cv2.SimpleBlobDetector_Params()  # https://stackoverflow.com/questions/8076889/how-to-use-opencv-simpleblobdetector
        self.fill_blob_detector_params()
        self.simple_blob_detector = cv2.SimpleBlobDetector_create(self.blob_detector_params)
        # robot position in the projected ground plane,
        # below the center of the image by some distance (mm)
        self.ground_w, self.ground_h = 1250, 1250
        self.robot_x, self.robot_y = self.ground_w / 2, self.ground_h + 100
        self.cam_w, self.cam_h = 640, 480


        # define other variables as needed
        self.img = None
        self.other_bot_coord = None
        self.lane_error = None



        # color to BGR dictionary
        self.color_to_bgr = {
            Color.RED : (0, 0, 255),
            Color.BLUE: (255, 0, 0),
            Color.GREEN: (0, 255, 0),
            Color.WHITE: (255, 255, 255),
            Color.YELLOW: (0, 255, 255),
            Color.BLACK: (0, 0, 0),
            Color.ORANGE: (0, 165, 255),
        }

        # flags
        self.stop_flag = False

        self.camera_sub = rospy.Subscriber(f"/{self.vehicle_name}/camera_node/image/compressed", CompressedImage, self.image_callback)
        self.car_cmd = rospy.Publisher(f"/{self.vehicle_name}/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1)
        self.lane_error_topic = rospy.Subscriber(f"/{self.vehicle_name}/lane_error", String, self.lane_error_callback)
        self.circle_img_pub = rospy.Publisher(f"/{self.vehicle_name}/circle_img", Image, queue_size=1)
        self.other_bot_info_pub = rospy.Publisher(f"/{self.vehicle_name}/other_bot_info", String, queue_size=1)
        self.lane_error_topic = rospy.Publisher(f"/{self.vehicle_name}/lane_error", String, queue_size=1)

        # move node services
        rospy.wait_for_service(f'/{self.vehicle_name}/drive_straight')
        rospy.wait_for_service(f'/{self.vehicle_name}/rotate')
        rospy.wait_for_service(f'/{self.vehicle_name}/drive_arc')
        self.rotate_request = rospy.ServiceProxy(f'/{self.vehicle_name}/rotate', SetString)
        self.drive_straight_request = rospy.ServiceProxy(f'/{self.vehicle_name}/drive_straight', SetString)
        return

    def lane_error_callback(self, msg):
        '''
        lane_error = {
            "lane_error": error
        }
        '''
        meas_json = msg.data
        self.lane_error = json.loads(meas_json)["lane_error"]

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
        if now - self.last_stamp < self.publish_duration:  
            return
        else:
            self.last_stamp = now

        image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        # undistort the image
        undistorted_image = self.undistort_image(image_cv)


        # crop the image to the center
        #undistorted_image = undistorted_image[100:400, 100:540]
        self.img = undistorted_image


        blob_points = self.detect_bot(undistorted_image)  # [(x, y), ...]

        if blob_points is None: return
        # draw the points on the image



        other_bot_coord = self.get_other_bot_coord(blob_points)

        for point in blob_points:
            cv2.circle(self.img, tuple(map(int, point)), 5, (0, 255, 0), -1)
        
        # draw the other bot coord
        self.put_text(self.img, f"Other bot coord: {other_bot_coord}", (10, 10))

        self.circle_img_pub.publish(self.bridge.cv2_to_imgmsg(undistorted_image, encoding="bgr8"))
        # msg 
        other_bot_msg = {
            "other_bot_coord": other_bot_coord,  # x, y of the other bot relative to the bot
        }
        rospy.loginfo(f"Other bot coord: {other_bot_coord}")
        json_le = json.dumps(str(other_bot_msg))
        self.other_bot_info_pub.publish(json_le)
        self.other_bot_coord = other_bot_coord
        return




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

        if centers is not None and centers.shape[0] == 21:  # dim (21, 1, 2)
            # good 
            centers = centers.squeeze()  # dim (21, 2)

            list_of_tuples = [tuple(point) for point in centers]

            return list_of_tuples   # ndarray
        else:
            blob_points = self.simple_blob_detector.detect(image_cv) # get the points
            # remove outliers
            blob_points = self.remove_outliers(blob_points, distance_threshold=50)

            if len(blob_points) == 0: return None
            return blob_points  # list of (x, y) tuples
    """
    
    Args:
        circle_points: list of (x, y) tuples of the circle grid centers
        we removed the outliers from the circle_points
    
    Return
        python list of tuples (x, y) of centers
    """
    def get_other_bot_coord(self, circle_points):
        assert circle_points is not None

        # get circle grid dim 
        (grid_width, grid_height) = self.approximate_duckiebot_circlegrid_dim(circle_points)

        self.put_text(self.img, f"Grid width: {grid_width}, Grid height: {grid_height}", (10, 30))

        if len(circle_points) == 21:
            # distance of the centers
            middle_column_center = circle_points[17] # 11th center is the middle column center of the 7x3 grid
            # project the middle point to the ground
            #proj_middle = self.project_points2([middle_column_center[0], middle_column_center[1] + 3.5*grid_height]) 
            proj_middle = self.project_point_to_ground([middle_column_center[0], middle_column_center[1] + 3.5*grid_height]) 
            cv2.circle(self.img, tuple(map(int, (middle_column_center[0], middle_column_center[1] + 3.5*grid_height))), 5, (0, 0, 255), -1)
        else:
            assert len(circle_points) > 0
            # get the mean of the points
            mean_point = np.mean(circle_points, axis=0) # dim = 2

            # draw mean point
            cv2.circle(self.img, tuple(map(int, [mean_point[0], mean_point[1] + 3.5*grid_height])), 5, (0, 0, 255), -1)
            #proj_middle = self.project_points2([mean_point[0], mean_point[1] + 3.5*grid_height])
            proj_middle = self.project_point_to_ground([mean_point[0], mean_point[1] + 3.5*grid_height])
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
        circle_points = [tuple(p) for p in circle_points]
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
        good = True
        for i in range(len(points)):
            for j in range(len(points)):
                if i == j:
                    continue
                if np.linalg.norm(points[i] - points[j]) > distance_threshold:
                    good = False
                    break

            if good:
                good_points.append(points[i])
            
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
        self.car_cmd.publish(Twist2DStamped(v=0, omega=0))
        while not rospy.is_shutdown():
            if self.other_bot_coord is None: continue  # wait for the other bot coord

            # find the circle grid
            if self.other_bot_coord[1] > 0 and self.other_bot_coord[1] < 200:
                # stop the bot
                self.car_cmd.publish(Twist2DStamped(v=0, omega=0))
                # sleep for 3 seconds
                break
            #self.circle_img_pub.publish(self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8"))
            # otherwise drive straight
            self.car_cmd.publish(Twist2DStamped(v=0.2, omega=0))
            rate.sleep()
        # sleep for 3 seconds
        rospy.sleep(3)
        self.overtake()


    def on_shutdown(self):
        # on shutdown,
        self.car_cmd.publish(Twist2DStamped(v=0, omega=0))
        pass
    
    def overtake(self):
        import math
        # rotate pi/4
        r_params = {
            "radians": math.pi/4,
            "speed": 2,
            "leds": False
        }
        self.rotate_request(json.dumps(r_params))

        rospy.loginfo("Rotated pi/4")
        rospy.sleep(1)

        # drive straight
        s_params = {
            "speed": 0.5,
            "meters": 0.3,
            "leds": False
        }
        self.drive_straight_request(json.dumps(s_params))

        rospy.loginfo("Drove straight 0.2m")
        rospy.sleep(1)
        r_params["radians"] = -math.pi/4


        self.rotate_request(json.dumps(r_params))
        return

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
    
    # from extrinsic
    def project_points2(self, point):
        x, y = point

        point = np.array([x, y, 1])

        ground_point = np.dot(self.homography, point)
        ground_point /= ground_point[2]  # normalize by z
        
        return ground_point[:2]
    

if __name__ == '__main__':
    # create the node
    rospy.sleep(2)
    node = VehicleDetection(node_name='april_tag_detector')
    #node.loop()
    node.overtake()
    rospy.spin()

"""
Circle grid indices
(0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6)  # First row
(1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6)  # Second row
(2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6)  # Third row

"""
