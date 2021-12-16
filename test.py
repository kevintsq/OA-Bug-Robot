import cv2 as cv

from robot_following_line import Robot


cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 160)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 120)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
with Robot() as robot:
    while True: 
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        cropped = frame[60:, :]
        # Our operations on the frame come here
        gray = cv.cvtColor(cropped, cv.COLOR_BGR2GRAY)
        _, thresh = cv.threshold(gray, 60, 255, cv.THRESH_BINARY_INV)  # Auto thresholding)
        # Display the resulting frame
        contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            cv.drawContours(cropped, contours, -1, (0, 255, 0), 1)
            M = cv.moments(contours[0])
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            cv.line(cropped, (cx, 0), (cx, 80), (0, 0, 255), 1)
            cv.line(cropped, (0, cy), (120, cy), (0, 0, 255), 1)

            if cx > 120:
                robot.turn_right()
            elif cx < 40:
                robot.turn_left()
            else:
                robot.go_front()
        else:
            robot.stop(0)
        cv.imshow('frame', cropped)
        if cv.waitKey(1) == ord('q'):
            break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
