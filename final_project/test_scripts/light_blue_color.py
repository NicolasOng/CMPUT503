import cv2
import numpy as np
import argparse

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


# projection to ground plane homography matrix
cam_w, cam_h = 640, 480
ground_w, ground_h = 1250, 1250
dst_pts_translation = np.array([(ground_w / 2) - 24, ground_h - 255], dtype=np.float32)
src_pts = np.array([[284, 285], [443, 285], [273, 380], [584, 380]], dtype=np.float32)
dst_pts = np.array([[0, 0], [186, 0], [0, 186], [186, 186]], dtype=np.float32)
dst_pts = dst_pts + dst_pts_translation
homography_to_ground, _ = cv2.findHomography(src_pts, dst_pts)


def undistort_image(cv2_img):
        h, w = cv2_img.shape[:2]
        # optimal camera matrix lets us see entire camera image (image edges cropped without), but some distortion visible
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_matrix, dist_coeff, (w,h), 0, (w,h))

        # undistorted image using calibration parameters
        return cv2.undistort(cv2_img, cam_matrix, dist_coeff, None)

def get_color_mask_custom(lower, upper, cv2_img):
    hsv_frame = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)

    # Create color mask
    color_mask = cv2.inRange(hsv_frame, lower, upper)
    color_mask = cv2.dilate(color_mask, kernel)
         
    return color_mask

def convert_hsv(hue, saturation, value):
    '''
    from custom:
    Hue: [0, 355]
    Saturation: [0, 100]
    Value: [0, 100]
    to opencv:
    Hue: [0,179]
    Saturation: [0,255]
    Value: [0,255]
    '''
    # Convert hue from [0, 355] to [0, 179]
    ocv_hue = int((hue / 355) * 179)

    # Convert saturation and value from [0, 100] to [0, 255]
    ocv_saturation = int((saturation / 100) * 255)
    ocv_value = int((value / 100) * 255)

    return [ocv_hue, ocv_saturation, ocv_value]

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    # Positional arguments (required)
    parser.add_argument("input", help="Path to the input file")

    # Parse and run
    args = parser.parse_args()

    image = cv2.imread(f"photos/{args.input}.png")
    image = undistort_image(image)

    lower = np.array(convert_hsv(140, 40, 20), np.uint8)
    upper = np.array(convert_hsv(240, 100, 100), np.uint8)
    mask = get_color_mask_custom(lower, upper, image)

    result_img = cv2.bitwise_and(image, image, mask=mask)

    #image = perform_ground_color_detection(image.copy(), image)

    cv2.imshow("PNG Image", result_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
