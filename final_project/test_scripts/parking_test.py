import apriltag
import cv2
import numpy as np

class CameraDetectionNode():
    def __init__(self, node_name):       
        self.draw_atag_toggle = True

        self.is_ToI = False
        self.ToI_area = 0

        self.parking_tag = 228
        self.at_detector = apriltag.Detector()

    # Draws a bounding box and ID on an ApriltTag 
    def draw_atag_features(self, image, points, id, center, colour=(255, 100, 255)):
        h, w = image.shape[:2]
        tag_offset_error = str(center[0] - w//2)
        img = cv2.polylines(image, [points], True, colour, 5)
        img = cv2.putText(image, tag_offset_error, self.tag_center, cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,100,255), 1)
        img = cv2.line(image, (w//2, h//2), center, (255,100,255), 2)
        #img = cv2.putText(image, id, (center[0], center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.75, colour, 2)
        return img

    def perform_tag_detection(self, clean_image, draw_image):
        # Convert image to grayscale
        image_grey = cv2.cvtColor(clean_image, cv2.COLOR_BGR2GRAY)
        image_grey = cv2.GaussianBlur(image_grey, (5,5), 0)

        # ApriltTag detector
        results = self.at_detector.detect(image_grey)

        ToI_index = -1
        self.is_ToI = False
        self.ToI_area = 0

        if len(results) == 0:
            return draw_image
        else:
            for idx, r in enumerate(results):
                if r.tag_id == self.parking_tag:
                    ToI_index = idx
                    self.is_ToI = True

        if ToI_index != -1:
            ToI = results[ToI_index]
            ToI_center = ToI.center.astype(int)
            ToI_corners = np.array(ToI.corners, np.int32)
            ToI_corners = ToI_corners.reshape((-1, 1, 2))
            ToI_id = str(ToI.tag_id)

            self.tag_center = ToI_center
            
            # WEBCAM IMAGE COORDINATES: TOP LEFT is 0,0
            # DUCKIEBOT CAMERA COORDINATES ARE REVERSED: BOTTOM LEFT is 0,0
            tl = ToI.corners[0].astype(int)
            br = ToI.corners[2].astype(int)
            ToI_area = (br[0] - tl[0]) * (br[1] - tl[1])
            self.ToI_area = ToI_area
            
            if self.draw_atag_toggle:
                self.draw_atag_features(draw_image, ToI_corners, ToI_id, ToI_center)

        #self.tag_list.publish(json.dumps(tags_list))
                    
        return draw_image
    
    def detection_loop(self):
        cap = cv2.VideoCapture(0)

        while True:
            _, clean_image = cap.read()

            h, w = clean_image.shape[:2]
            draw_image = clean_image.copy()
            draw_image = self.perform_tag_detection(clean_image, draw_image)

            if self.ToI_area > 115000:
                print("Area threshold reached")

            # Center line of image
            draw_image = cv2.line(draw_image, (w//2, 0), (w//2, h), (0,255,0), 2)
            draw_image = cv2.circle(draw_image, (w//2, h//2), 3, (0,255,0), 3)

            cv2.imshow('Camera Detection', draw_image)

            key = cv2.waitKey(5)
            if key == ord('q'):
                break

        print("[PARKING_TEST.PY] Terminate")

if __name__ == '__main__':
    node = CameraDetectionNode(node_name='camera_detection_node')
    node.detection_loop()

