import cv2 as cv

from robot import Robot

import numpy as np


opening_kernel_size = 5
opening_kernel = np.ones((opening_kernel_size, opening_kernel_size), np.uint8)
closing_kernel_size = 25
closing_kernel = np.ones((closing_kernel_size, closing_kernel_size), np.uint8)


def getROI(image):
    hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    lowMat = cv.inRange(hsv_image, np.array([0, 0, 0]), np.array([120, 120, 90]))
    kernel = cv.getStructuringElement(cv.MORPH_RECT,(40, 40))
    closed = cv.morphologyEx(lowMat, cv.MORPH_CLOSE, kernel)
    return closed


def getLine(gray):
    edges = cv.Canny(gray,50,120)
    lines = cv.HoughLinesP(edges, 1, np.pi/180, 100, lines=100, minLineLength=20,maxLineGap=120)
    lines = lines[:,0,:]
    sumLength = 0
    sumThetaLength = 0
    for x1,y1,x2,y2 in lines:
        theta = np.rad2deg(np.arctan2(abs(y2-y1),abs(x2-x1)))
        length = np.linalg.norm([x2-x1,y2-y1])
        sumThetaLength += theta*length
        sumLength += length
        print(x1,y1,x2,y2,theta,length)
    return sumThetaLength/sumLength


cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 160)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 120)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
with Robot() as robot:
    while True: 
        # Capture frame-by-frame
        ret, img_src = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        cropped = img_src[60:, :]
        img_grey = cv.cvtColor(cropped, cv.COLOR_BGR2GRAY)
        ret, img_thresh = cv.threshold(img_grey, 60, 255, cv.THRESH_BINARY_INV)  # Auto thresholding
        img_thresh = cv.morphologyEx(img_thresh, cv.MORPH_OPEN, opening_kernel)  # Remove background dots
        img_thresh = cv.morphologyEx(img_thresh, cv.MORPH_CLOSE, closing_kernel)  # Remove foreground dots
        contours, _ = cv.findContours(img_thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # if len(contours) > 0:
        #     grayROI = getROI(cropped)
        #     theta = getLine(grayROI)
        #     print(theta)
        cv.imshow('frame', cropped)
        if cv.waitKey(1) == ord('q'):
            break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
