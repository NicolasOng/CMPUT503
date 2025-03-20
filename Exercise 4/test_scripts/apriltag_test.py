import cv2 as cv
import apriltag

if __name__ == '__main__':
    
    img = cv.imread('atag.jpeg')
    img_grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    image_grey = cv2.GaussianBlur(image_grey, (5,5), 0)

    detector = apriltag.Detector()
    results = detector.detect(img_grey)

    tags_list = []

    largest_tag_index = 0
    largest_tag_area = 0
    
    '''
    # If multiple tags detected, find most prominent tag (largest by area)
    if len(results) > 1:
        for idx, r in enumerate(results):
            tl = r.corners[0].astype(int)
            br = r.corners[2].astype(int)
            area = (tl[1] - br[1]) * (br[0] - tl[0])
            if area > largest_tag_area:
                largest_tag_index = idx
                largest_tag_area = area
    '''
    
    print("Number of detected tags: ", len(results))
    count = 0 
    for r in results:
        print(r.corners)
        top_left = r.corners[0].astype(int)
        bottom_right = r.corners[2].astype(int)
        if count % 2 == 1:
            rect = cv.rectangle(img, bottom_right, top_left, (255, 100, 100), 5)
        else: 
            rect = cv.rectangle(img, bottom_right, top_left, (100, 255, 100), 5)
        count += 1

    rect = cv.resize(rect, (960, 540))
    cv.imshow('test', rect)
    cv.waitKey(0)
    cv.destroyAllWindows()