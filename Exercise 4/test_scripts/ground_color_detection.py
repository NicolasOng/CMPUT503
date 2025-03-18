import cv2
import numpy as np
import argparse

from Color import Color

# camera matrix and distortion coefficients from intrinsic.yaml file
cam_matrix = np.array([
    [319.2461317458548, 0.0, 307.91668484581703],
    [0.0, 317.75077109798957, 255.6638447529814],
    [0.0, 0.0, 1.0]
])
dist_coeff = np.array([-0.25706255601943445, 0.045805679651939275, -0.0003584336283982042, -0.0005756902051068707, 0.0])

# from extrinsic.yaml file
homography = np.array([
    [-0.00013668875104344582, 0.0005924050290243054, -0.5993724660928124],
    [-0.0022949507610645035, -1.5331615246117395e-05, 0.726763100835842],
    [0.00027302496335237673, 0.017296161892938217, -2.946528752705874]
])

# Target color in HSV
red_hsv = np.array([0, 144, 255])  # rGB: 255, 111, 111




# Color Detection Stuff
# color detection parameters in HSV format
red_lower = np.array([160, 255/3, 255/2], np.uint8)
red_upper = np.array([180, 255/3, 255/3], np.uint8)

# Define range
hr = 40
sr = 50
vr = 50

red_lower = np.array([max(0, red_hsv[0] - hr), max(0, red_hsv[1] - sr), max(0, red_hsv[2] - vr)])
red_upper = np.array([min(179, red_hsv[0] + hr), min(255, red_hsv[1] + sr), min(255, red_hsv[2] + vr)])

# Wrap-around case (near 0)
red_lower_1 = np.array([0, max(0, red_hsv[1] - sr), max(0, red_hsv[2] - vr)])
red_upper_1 = np.array([red_hsv[0] + hr, min(255, red_hsv[1] + sr), min(255, red_hsv[2] + vr)])

red_lower_2 = np.array([179 - hr + red_hsv[0], max(0, red_hsv[1] - sr), max(0, red_hsv[2] - vr)])
red_upper_2 = np.array([179, min(255, red_hsv[1] + sr), min(255, red_hsv[2] + vr)])

#red_lower = red_lower_1
#red_upper = red_upper_1

#red_lower = np.array([136, 87, 111], np.uint8)
#red_upper = np.array([180, 255, 255], np.uint8)


green_lower = np.array([34, 52, 72], np.uint8)
green_upper = np.array([82, 255, 255], np.uint8)

blue_lower = np.array([105, 80, 120], np.uint8)
blue_upper = np.array([135, 255, 255], np.uint8)

yellow_lower = np.array([21, 100, 60*2.55], np.uint8)
yellow_higher = np.array([33, 255, 100*2.55], np.uint8)

white_lower = np.array([0, 0, 200], np.uint8)
white_higher = np.array([180, 40, 255], np.uint8)

# color bounds
color_bounds = {
    Color.RED: (red_lower, red_upper),
    Color.BLUE: (blue_lower, blue_upper),
    Color.GREEN: (green_lower, green_upper),
    Color.YELLOW: (yellow_lower, yellow_higher),
    Color.WHITE: (white_lower, white_higher),
}

# color to BGR dictionary
color_to_bgr = {
    Color.RED : (0, 0, 255),
    Color.BLUE: (255, 0, 0),
    Color.GREEN: (0, 255, 0),
    Color.WHITE: (255, 255, 255),
    Color.YELLOW: (0, 255, 255),
    Color.BLACK: (0, 0, 0),
}

# projection to ground plane homography matrix
cam_w, cam_h = 640, 480
ground_w, ground_h = 1250, 1250
dst_pts_translation = np.array([(ground_w / 2) - 24, ground_h - 255], dtype=np.float32)
src_pts = np.array([[284, 285], [443, 285], [273, 380], [584, 380]], dtype=np.float32)
dst_pts = np.array([[0, 0], [186, 0], [0, 186], [186, 186]], dtype=np.float32)
dst_pts = dst_pts + dst_pts_translation
homography_to_ground, _ = cv2.findHomography(src_pts, dst_pts)

# robot position in the projected ground plane,
# below the center of the image by some distance (mm)
robot_x, robot_y = ground_w / 2, ground_h + 100

horizon = cam_h * 0.6

offset = 0
polygon_points = np.array([
    [60 * cam_w // 100, cam_h/2],   # Top-right
    [40 * cam_w // 100, cam_h/2],       # Top-left
    [5 * cam_w // 100, cam_h],       # bottom-left
    [95 * cam_w // 100, cam_h],   # bottom-right
], np.int32)
lane_mask = np.zeros((cam_h, cam_w), dtype=np.uint8)
cv2.fillPoly(lane_mask, [polygon_points], 255)

polygon_points_white = np.array([
    [55 * cam_w // 100, cam_h/2],   # Top-right
    [45 * cam_w // 100, cam_h/2],       # Top-left
    [20 * cam_w // 100, cam_h],       # bottom-left
    [80 * cam_w // 100, cam_h],   # bottom-right
], np.int32)
lane_mask_white = np.zeros((cam_h, cam_w), dtype=np.uint8)
cv2.fillPoly(lane_mask_white, [polygon_points_white], 255)

def undistort_image(cv2_img):
        h, w = cv2_img.shape[:2]
        # optimal camera matrix lets us see entire camera image (image edges cropped without), but some distortion visible
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_matrix, dist_coeff, (w,h), 0, (w,h))

        # undistorted image using calibration parameters
        return cv2.undistort(cv2_img, cam_matrix, dist_coeff, None)

def get_color_mask(color: Color, cv2_img):
    '''
    the color mask gets all the pixels in the image that are within the color bounds
    the color mask is an ndarray of shape (h, w) with values 0 or 255.
    0 means the pixel is not in the color bounds, 255 means it is
    '''
    hsv_frame = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)

    # Get the lower and upper bounds for the given color
    lower, upper = color_bounds.get(color, (None, None))
    assert lower is not None and upper is not None, f"Invalid color: {color}"

    # Create color mask
    color_mask = cv2.inRange(hsv_frame, lower, upper)
    color_mask = cv2.dilate(color_mask, kernel)
    '''
    if (color == Color.RED):
        lower, upper = red_lower_2, red_upper_2
        # Create color mask
        color_mask1 = cv2.inRange(hsv_frame, lower, upper)
        color_mask1 = cv2.dilate(color_mask, kernel)
        color_mask = cv2.bitwise_or(color_mask, color_mask1)
    #'''

    color_mask = cv2.bitwise_and(color_mask, lane_mask)
    if color == Color.WHITE:
        color_mask = cv2.bitwise_and(color_mask, lane_mask_white)
         
    return color_mask

def get_contours(color_mask):
    '''
    using the color mask, we can get the contours of the color
    the contours are the edges of the color, defined by a list of points
    contours is a tuple of ndarrays of shape (n, 1, 2)
    '''
    contours, hierarchy = cv2.findContours(color_mask, 
                                        cv2.RETR_TREE, 
                                        cv2.CHAIN_APPROX_SIMPLE)
    return contours, hierarchy

def get_bounding_boxes(color, cv2_img):
    # get the color mask
    color_mask = get_color_mask(color, cv2_img)
    # get the color contours
    contours, hierarchy = get_contours(color_mask)
    # get the nearest bounding box (to the bottom middle of the image)
    bbs = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if (area > 300): 
            x, y, w, h = cv2.boundingRect(contour)
            contour_bb = (x, y, w, h)
            contour_center = (x + w / 2, y + h / 2)
            if contour_center[1] > horizon:
                    bbs.append({"bb": contour_bb, "center": contour_center})
    return bbs

def project_point_to_ground(point):
    '''
    point is a tuple of (x, y) coordinates
    the point is relative to the bot.
    '''
    point = np.array([point], dtype=np.float32)
    new_point = cv2.perspectiveTransform(point.reshape(-1, 1, 2), homography_to_ground)
    new_point = new_point.ravel()
    new_point = (new_point[0] - robot_x, -(new_point[1] - robot_y))
    return new_point

def project_bounding_box_to_ground(bounding_box):
    '''
    bounding_box is a tuple of (x, y, w, h) coordinates
    output is a list of 4 points in the ground plane.
    These points are relative to the robot.
    '''
    if not bounding_box: return None
    x, y, w, h = bounding_box
    points = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]], dtype=np.float32)
    new_points = cv2.perspectiveTransform(points.reshape(-1, 1, 2), homography_to_ground)
    new_points = new_points.reshape(-1, 2)
    transformed_coords = np.column_stack([
        new_points[:, 0] - robot_x,
        -(new_points[:, 1] - robot_y)
    ])
    return transformed_coords

def project_bounding_boxes_to_ground(bbs):
    pbbs = []
    for bb in bbs:
        pbb = project_bounding_box_to_ground(bb["bb"])
        pc = project_point_to_ground(bb["center"])
        pbbs.append({"bb": pbb, "center": pc})
    return pbbs

def draw_bounding_box(image, bb, center, coords, color):
    '''
    this function draws the bounding box and the ground x, y coordinates
    '''
    # draw the bounding box
    x, y, w, h = bb
    cv2.rectangle(image, (x, y), (x + w, y + h), color_to_bgr[color], 2) 
    # draw the center
    cv2.circle(image, (int(center[0]), int(center[1])), radius=2, color=color_to_bgr[color], thickness=-1)
    # draw the x, y coordinates
    cv2.putText(image, f"({coords[0]:.2f}, {coords[1]:.2f})", (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_to_bgr[color])


def draw_bounding_boxes_to_image(image, bbs, projected_bbs, color):
    for i in range(len(bbs)):
        bb = bbs[i]
        pbb = projected_bbs[i]
        draw_bounding_box(image, bb["bb"], bb["center"], pbb["center"], color)
        
def perform_ground_color_detection(clean_image, draw_image):
    # get all the bounding boxes above a certain area for each color (white, red, blue)
    # also in the bottom half of the image
    red_bbs = get_bounding_boxes(Color.RED, clean_image)
    white_bbs = get_bounding_boxes(Color.WHITE, clean_image)
    blue_bbs = get_bounding_boxes(Color.BLUE, clean_image)
    # project the bounding boxes (and their centers) to the ground
    red_bbs_p = project_bounding_boxes_to_ground(red_bbs)
    white_bbs_p = project_bounding_boxes_to_ground(white_bbs)
    blue_bbs_p = project_bounding_boxes_to_ground(blue_bbs)
    # publish the info to the topic
    color_coords = {
        "red": red_bbs_p,
        "white": white_bbs_p,
        "blue": blue_bbs_p
    }
    print(color_coords)
    # draw the bounding boxes and their centers, with their center ground coordinates
    draw_bounding_boxes_to_image(draw_image, red_bbs, red_bbs_p, Color.RED)
    draw_bounding_boxes_to_image(draw_image, white_bbs, white_bbs_p, Color.WHITE)
    draw_bounding_boxes_to_image(draw_image, blue_bbs, blue_bbs_p, Color.BLUE)
    return draw_image


parser = argparse.ArgumentParser()

# Positional arguments (required)
parser.add_argument("input", help="Path to the input file")

# Parse and run
args = parser.parse_args()

image = cv2.imread(f"camera/{args.input}.png")
image = undistort_image(image)
image = perform_ground_color_detection(image.copy(), image)

bgr_color = cv2.cvtColor(np.array([[red_lower]], dtype=np.uint8), cv2.COLOR_HSV2BGR)[0][0]
hsv_color = cv2.cvtColor(np.array([[[255, 111, 111]]], dtype=np.uint8), cv2.COLOR_RGB2HSV)
print(hsv_color) #0 144 255
#cv2.circle(image, (50, 50), 100, bgr_color.tolist(), -1)

cv2.line(image, tuple(polygon_points_white[0]), tuple(polygon_points_white[3]), (0, 255, 0), 2)  # Left line
cv2.line(image, tuple(polygon_points_white[1]), tuple(polygon_points_white[2]), (0, 255, 0), 2)  # Right line

cv2.imshow("PNG Image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
