import cv2
import numpy as np

from enum import Enum

#from a photo with the calibration thingy:
'''
284, 284 - 0, 0
443, 280 - 186mm, 0
273, 398 - 0, -186mm
584, 375 - 186, -186
'''

w, h = 1200, 900
translation = np.array([w / 2 - 93/2, 643], dtype=np.float32)
src_pts = np.array([[284, 285], [443, 285], [273, 380], [584, 380]], dtype=np.float32)
dst_pts = np.array([[0, 0], [186, 0], [0, 186], [186, 186]], dtype=np.float32)
dst_pts = dst_pts + translation
homography_to_ground, _ = cv2.findHomography(src_pts, dst_pts)

class Color:
    RED = 0
    BLUE = 1
    GREEN = 2
    YELLOW = 3
    WHITE = 4

# color detection parameters in HSV format
red_lower = np.array([136, 87, 111], np.uint8) 
red_upper = np.array([180, 255, 255], np.uint8) 

green_lower = np.array([34, 52, 72], np.uint8) 
green_upper = np.array([82, 255, 255], np.uint8) 

blue_lower = np.array([110, 80, 120], np.uint8) 
blue_upper = np.array([130, 255, 255], np.uint8) 

yellow_lower = np.array([21, 100, 60*2.55], np.uint8)
yellow_higher = np.array([33, 255, 100*2.55], np.uint8)

white_lower = np.array([0, 0, 200], np.uint8)  # for white. any value of Hue works. just maximum brighteness
white_higher = np.array([170, 25, 255], np.uint8)

color_to_bgr = {
        Color.RED : (0, 0, 255),
        Color.BLUE: (255, 0, 0),
        Color.GREEN: (0, 255, 0),
        Color.WHITE: (255, 255, 255),
        Color.YELLOW: (0, 255, 255),
    }  

color_to_str = {
            Color.RED : "red",
            Color.BLUE: "blue",
            Color.GREEN: "green",
            Color.WHITE: "white",
            Color.YELLOW: "yellow",
        }
# camera matrix and distortion coefficients from intrinsic.yaml file
cam_matrix = np.array([[319.2461317458548, 0.0, 307.91668484581703], [0.0, 317.75077109798957, 255.6638447529814], [0.0, 0.0, 1.0]])
dist_coeff = np.array([-0.25706255601943445, 0.045805679651939275, -0.0003584336283982042, -0.0005756902051068707, 0.0])
        
# from extrinsic.yaml file
homography = np.array([[-0.00013668875104344582, 0.0005924050290243054, -0.5993724660928124], [-0.0022949507610645035, -1.5331615246117395e-05, 0.726763100835842], [0.00027302496335237673, 0.017296161892938217, -2.946528752705874]])


def undistort_image(cv2_img):
    # add your code here
    h, w = cv2_img.shape[:2]
    # optimal camera matrix lets us see entire camera image (image edges cropped without), but some distortion visible
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_matrix, dist_coeff, (w,h), 0, (w,h))
    # undistorted image using calibration parameters
    undistorted_cv2img = cv2.undistort(cv2_img, cam_matrix, dist_coeff, None)
    #undistorted_cv2img = cv2.cvtColor(undistorted_cv2img, cv2.COLOR_BGR2RGB)
    return undistorted_cv2img

def get_color_mask(color: Color, cv2_img):
    hsv_frame = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)

    # Define color bounds mapping
    color_bounds = {
        Color.RED: (red_lower, red_upper),
        Color.BLUE: (blue_lower, blue_upper),
        Color.GREEN: (green_lower, green_upper),
        Color.YELLOW: (yellow_lower, yellow_higher),
        Color.WHITE: (white_lower, white_higher),
    }

    # Get the lower and upper bounds for the given color
    lower, upper = color_bounds.get(color, (None, None))
    assert lower is not None and upper is not None, f"Invalid color: {color}"

    # Create color mask
    color_mask = cv2.inRange(hsv_frame, lower, upper)
    color_mask = cv2.dilate(color_mask, kernel)

    return color_mask

def project_points_to_ground(point):
    x, y = point

    point = np.array([x, y, 1])

    ground_point = np.dot(homography, point)
    ground_point /= ground_point[2]  # normalize by z
    
    return ground_point[:2]

def draw_contour(color: Color, cv2_img):
    color_mask = get_color_mask(color, cv2_img)
    color_bgr = color_to_bgr[color]  # (0-255, 0-255, 0-255) bgr format

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
    return cv2_img

def combine_masks(mask1, mask2):
    return cv2.bitwise_or(mask1, mask2)

def apply_mask(img, mask):
    return cv2.bitwise_and(img, img, mask=mask) 

def project_image_to_ground(image):
    h, w = image.shape[:2]

    # Compute new bounding box size after transformation (optional)
    corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32)
    new_corners = cv2.perspectiveTransform(corners.reshape(-1, 1, 2), homography_to_ground)

    print(new_corners)
    
    # Compute the new width and height
    x_min, y_min = new_corners.min(axis=0).ravel()
    x_max, y_max = new_corners.max(axis=0).ravel()
    
    new_w, new_h = int(x_max - x_min), int(y_max - y_min)

    # Apply perspective warp
    warped_image = cv2.warpPerspective(image, homography_to_ground, (1200, 900), flags=cv2.INTER_CUBIC)

    return warped_image


if __name__ == "__main__":
    image = cv2.imread("camera/image05.png")
    image = undistort_image(image)
    image = project_image_to_ground(image)
    #image = draw_contour(Color.YELLOW, image)
    yellow_mask = get_color_mask(Color.YELLOW, image)
    white_mask = get_color_mask(Color.WHITE, image)
    mask = combine_masks(yellow_mask, white_mask)
    image = apply_mask(image, mask)
    cv2.imshow("PNG Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    #detect_lane_color(cv2.imread("camera/image01.png"))