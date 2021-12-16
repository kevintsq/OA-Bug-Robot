import cv2 as cv
import numpy as np
import math

# img = cv.imread("red.jpg")
# if img is None:
#     exit("Could not read the image.")

window_name = 'Edge Map'
trackbar_title = 'Min Threshold'
canny_max_low_threshold = 300
canny_low_thresh_ratio = 3
canny_kernel_size = 3
opening_kernel_size = 5
opening_kernel = np.ones((opening_kernel_size, opening_kernel_size), np.uint8)
closing_kernel_size = 25
closing_kernel = np.ones((closing_kernel_size, closing_kernel_size), np.uint8)
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 160)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 120)


def CannyThreshold(low_threshold):
    img_blur = cv.GaussianBlur(img_grey, (5, 5), 0)
    ret, img_thresh = cv.threshold(img_blur, 60, 255, cv.THRESH_BINARY_INV)  # Auto thresholding
    img_thresh = cv.morphologyEx(img_thresh, cv.MORPH_OPEN, opening_kernel)  # Remove background dots
    img_thresh = cv.morphologyEx(img_thresh, cv.MORPH_CLOSE, closing_kernel)  # Remove foreground dots
    detected_edges = cv.Canny(img_thresh, low_threshold, low_threshold * canny_low_thresh_ratio, canny_kernel_size)
    # mask = detected_edges != 0
    # dst = src * (mask[:, :, None].astype(src.dtype))
    contours, _ = cv.findContours(detected_edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    print(f"len(edge_contours) = {len(contours)}")
    for contour in contours:
        if len(contour) > 25:  # Can be changed
            S1 = cv.contourArea(contour)
            ellipse = cv.fitEllipse(contour)
            S2 = math.pi * ellipse[1][0] * ellipse[1][1]
            if S2 != 0 and (S1 / S2) > 0.2:  # Can be changed
                cv.ellipse(img_src, ellipse, color=(0, 255, 0), thickness=2)
                # print(str(S1) + "    " + str(S2) + "   " + str(ellipse[0][0]) + "   " + str(ellipse[0][1]))
    cv.drawContours(img_src, contours, -1, (0, 0, 255), 1)
    cv.imshow(window_name, img_src)
    if cv.waitKey(1) == ord("q"):
        cap.release()
        cv.destroyAllWindows()
        exit()


cv.namedWindow(window_name)
cv.createTrackbar(trackbar_title, window_name, 0, canny_max_low_threshold, CannyThreshold)
while True:
    _, img_src = cap.read()
    img_grey = cv.cvtColor(img_src, cv.COLOR_BGR2GRAY)
    CannyThreshold(cv.getTrackbarPos(trackbar_title, window_name))
