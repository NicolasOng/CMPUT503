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
translation = np.array([w / 2 - 50, 643], dtype=np.float32)
src_pts = np.array([[284, 285], [443, 285], [273, 380], [584, 380]], dtype=np.float32)
dst_pts = np.array([[0, 0], [186, 0], [0, 186], [186, 186]], dtype=np.float32)
dst_pts = dst_pts + translation
# this matrix makes it so 1px=1mm.
# should be easy to find distances, just define 0, 0 to be way below the image, and everything is relative to that.
homography_to_ground, _ = cv2.findHomography(src_pts, dst_pts)

class Color:
    RED = 0
    BLUE = 1
    GREEN = 2
    YELLOW = 3
    WHITE = 4
    BLACK = 5

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
        Color.BLACK: (0, 0, 0),
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

def rotate_image(image):
    '''
    this function rotates the image 90 degrees clockwise
    '''
    rotated_img = cv2.transpose(image)
    rotated_img = cv2.flip(rotated_img, flipCode=1)
    return rotated_img

def undistort_image(cv2_img):
    '''
    this function undistorts the image using the camera matrix and distortion coefficients
    these were calibrated manually for the given camera
    '''
    # add your code here
    h, w = cv2_img.shape[:2]
    # optimal camera matrix lets us see entire camera image (image edges cropped without), but some distortion visible
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_matrix, dist_coeff, (w,h), 0, (w,h))
    # undistorted image using calibration parameters
    undistorted_cv2img = cv2.undistort(cv2_img, cam_matrix, dist_coeff, None)
    #undistorted_cv2img = cv2.cvtColor(undistorted_cv2img, cv2.COLOR_BGR2RGB)
    return undistorted_cv2img

def get_color_mask(color: Color, cv2_img):
    '''
    the color mask gets all the pixels in the image that are within the color bounds
    the color mask is an ndarray of shape (h, w) with values 0 or 255.
    0 means the pixel is not in the color bounds, 255 means it is
    '''
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

def get_color_mask_pixel_list(color: Color, cv2_img):
    '''
    this function returns a list of all the pixels in the image that are within the color bounds
    the list is an ndarray of shape (n, 2) with n being the number of pixels
    '''
    # Get binary mask
    mask = get_color_mask(color, cv2_img)  # Shape (h, w), values 0 or 255

    # Get coordinates where mask is 255
    y_coords, x_coords = np.where(mask == 255)

    # Convert to (n, 2) shape in (x, y) order
    points = np.column_stack((x_coords, y_coords))

    return points  # Returns an (n, 2) array of (x, y) coordinates

def get_contours(color: Color, cv2_img):
    '''
    using the color mask, we can get the contours of the color
    the contours are the edges of the color, defined by a list of points
    contours is a tuple of ndarrays of shape (n, 1, 2)
    '''
    color_mask = get_color_mask(color, cv2_img)
    contours, hierarchy = cv2.findContours(color_mask, 
                                        cv2.RETR_TREE, 
                                        cv2.CHAIN_APPROX_SIMPLE)
    return contours

def draw_contour(color: Color, cv2_img):
    '''
    this function draws bounding boxes around the found contours
    of the given color
    '''
    contours = get_contours(color, cv2_img)
    color_bgr = color_to_bgr[color]  # (0-255, 0-255, 0-255) bgr format
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour) 
        if(area > 0): 
            x, y, w, h = cv2.boundingRect(contour) 
            cv2_img = cv2.rectangle(cv2_img, (x, y), 
                                    (x + w, y + h), 
                                    color_bgr, 2)     
    return cv2_img

def draw_points(points, color, cv2_img):
    for point in points:
        x, y = point
        cv2.circle(cv2_img, (int(x), int(y)), radius=2, color=color_to_bgr[color], thickness=-1)
    return cv2_img

def get_best_fit_line(points, degree=1):
    x = points[:, 0].flatten()
    y = points[:, 1].flatten()

    coeffs = np.polyfit(x, y, degree)

    if degree == 1:
        print(f"Best-fit line: y = {coeffs[0]:.3f}x + {coeffs[1]:.3f}")
    elif degree == 2:
        print(f"Best-fit curve: y = {coeffs[0]:.3f}x² + {coeffs[1]:.3f}x + {coeffs[2]:.3f}")

    return coeffs

def plot_best_fit_line(coeffs, cv2_img, color):
    # Generate x values for plotting in image
    height, width = cv2_img.shape[:2]
    x_fit = np.linspace(0, width, 1000)  # 100 points for smooth curve
    y_fit = np.polyval(coeffs, x_fit)  # Compute corresponding y values

    # Convert (x, y) into integer pixel coordinates
    curve_points = np.column_stack((x_fit, y_fit)).astype(np.int32)

    # Draw the curve on the OpenCV image
    cv2.polylines(cv2_img, [curve_points], isClosed=False, color=color_to_bgr[Color.BLACK], thickness=6)
    cv2.polylines(cv2_img, [curve_points], isClosed=False, color=color_to_bgr[color], thickness=2)

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
    
    # Compute the new width and height
    x_min, y_min = new_corners.min(axis=0).ravel()
    x_max, y_max = new_corners.max(axis=0).ravel()
    
    new_w, new_h = int(x_max - x_min), int(y_max - y_min)

    # Apply perspective warp
    warped_image = cv2.warpPerspective(image, homography_to_ground, (1200, 900), flags=cv2.INTER_CUBIC)

    return warped_image

def draw_vertical_line(image, x, color):
    '''
    draws a vertical line at the given x-coordinate
    '''
    cv2.line(image, (x, 0), (x, image.shape[0]), color=color_to_bgr[color], thickness=2)
    return image

def draw_horizontal_line(image, y, color):
    '''
    draws a horizontal line at the given y-coordinate
    '''
    cv2.line(image, (0, y), (image.shape[1], y), color=color_to_bgr[color], thickness=2)
    return image

def best_fit_line_rotated_filtered(color, image, degree=1, div_coeffs=None, above=False):
    '''
    this function gets and draws the best fit line for the given color,
    but in a rotated image, since we mostly deal with vertical lines
    (best fit lines don't deal with veritcal lines well)
    also filters out points that are farther in the distance from the camera
    '''
    # rotate the image 90 degrees clockwise
    image = rotate_image(image)
    # get the color mask pixels
    points = get_color_mask_pixel_list(color, image)
    # filter the pixels for ones to the left of a certain x-coordinate
    x_threshold = 400
    points = points[points[:, 0] < x_threshold]
    # draw that threshold line
    image = draw_vertical_line(image, x_threshold, color)
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
    # draw the filtered points
    image = draw_points(points, color, image)
    # get the best fit line
    coeffs = get_best_fit_line(points, degree=degree)
    # plot the best fit line
    image = plot_best_fit_line(coeffs, image, color)
    # optionally, rotate the image back
    image = rotate_image(image)
    image = rotate_image(image)
    image = rotate_image(image)
    return image, coeffs

def plot_best_fit_line_rotated(coeffs, image, color):
    '''
    this function plots the best fit line in the rotated image
    '''
    image = rotate_image(image)
    image = plot_best_fit_line(coeffs, image, color)
    image = rotate_image(image)
    image = rotate_image(image)
    image = rotate_image(image)
    return image

def get_mse(coeff_target, coeff_measured):
    '''
    this function calculates the mean squared error between two lines
    '''
    # get x-values for the bottom half of the image
    x_values = np.linspace(400, h, 100)
    # get y-values for both lines
    y_target = np.polyval(coeff_target, x_values)
    y_measured = np.polyval(coeff_measured, x_values)
    # Compute Mean Squared Error (MSE)
    mse = np.mean((y_measured - y_target) ** 2)
    return mse

def plot_errors_rotated(coeff_target, coeff_measured, image):
    '''
    this function plots the error between the target line and the measured line
    '''
    image = rotate_image(image)
    # get x-values for the bottom half of the image
    x_values = np.linspace(0, 400, 100)
    # get y-values for both lines
    y_target = np.polyval(coeff_target, x_values)
    y_measured = np.polyval(coeff_measured, x_values)
    # get the error
    errors = y_measured - y_target
    # plot the errors
    for x, e in zip(x_values, errors):
        x = int(x)
        e = int(e)
        cv2.line(image, (int(x), 600), (x, 600 + e), color=color_to_bgr[Color.RED], thickness=1)
    image = rotate_image(image)
    image = rotate_image(image)
    image = rotate_image(image)
    return image

if __name__ == "__main__":
    degree = 2
    image = cv2.imread("camera/image02.png")
    image = undistort_image(image)
    image = project_image_to_ground(image)
    image, yellow_line = best_fit_line_rotated_filtered(Color.YELLOW, image, degree=degree)
    image, white_line = best_fit_line_rotated_filtered(Color.WHITE, image, degree=degree, div_coeffs=yellow_line, above=True)
    measured_line = (np.array(yellow_line) + np.array(white_line)) / 2
    image = plot_best_fit_line_rotated(measured_line, image, Color.RED)
    target_line = [0, 600]
    if degree == 2:
        target_line = [0, 0, 600]
    image = plot_best_fit_line_rotated(target_line, image, Color.GREEN)
    image = plot_errors_rotated(target_line, measured_line, image)
    cv2.imshow("PNG Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
