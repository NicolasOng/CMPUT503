import cv2 as cv
import apriltag

if __name__ == '__main__':
    
    img = cv.imread('atag.jpeg')
    img_grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    detector = apriltag.Detector()
    results = detector.detect(img_grey)

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