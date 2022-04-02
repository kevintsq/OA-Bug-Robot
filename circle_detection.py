import cv2
import numpy as np


kernel = np.ones((5, 5), np.uint8)

cap = cv2.VideoCapture(0)


def nothing(_):
    pass


# 创建一个供以后使用的窗口
cv2.namedWindow('HueComp')
cv2.namedWindow('SatComp')
cv2.namedWindow('ValComp')
cv2.namedWindow('closing')
cv2.namedWindow('tracking')

# 创建跟踪条的最小和最大的色调，饱和度和价值
# 您可以根据需要调整默认值
cv2.createTrackbar('hmin', 'HueComp', 12, 179, nothing)
cv2.createTrackbar('hmax', 'HueComp', 37, 179, nothing)

cv2.createTrackbar('smin', 'SatComp', 96, 255, nothing)
cv2.createTrackbar('smax', 'SatComp', 255, 255, nothing)

cv2.createTrackbar('vmin', 'ValComp', 186, 255, nothing)
cv2.createTrackbar('vmax', 'ValComp', 255, 255, nothing)


while True:
    _, frame = cap.read()

    # 转换到HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)

    # 获取信息从轨道酒吧和多愁善感的结果
    hmn = cv2.getTrackbarPos('hmin', 'HueComp')
    hmx = cv2.getTrackbarPos('hmax', 'HueComp')

    smn = cv2.getTrackbarPos('smin', 'SatComp')
    smx = cv2.getTrackbarPos('smax', 'SatComp')

    vmn = cv2.getTrackbarPos('vmin', 'ValComp')
    vmx = cv2.getTrackbarPos('vmax', 'ValComp')

    # 应用阈值
    hthresh = cv2.inRange(np.array(hue), np.array(hmn), np.array(hmx))
    sthresh = cv2.inRange(np.array(sat), np.array(smn), np.array(smx))
    vthresh = cv2.inRange(np.array(val), np.array(vmn), np.array(vmx))

    # h s和v
    tracking = cv2.bitwise_and(hthresh, cv2.bitwise_and(sthresh, vthresh))

    # 一些morpholigical过滤
    dilation = cv2.dilate(tracking, kernel, iterations=1)
    closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
    closing = cv2.GaussianBlur(closing, (5, 5), 0)

    # 使用HoughCircles检测圆
    circles = cv2.HoughCircles(closing, cv2.HOUGH_GRADIENT, 2, 120, param1=120, param2=50, minRadius=10, maxRadius=0)
    # circles = np.uint16(np.around(circles))

    # 画圆圈
    if circles is not None:
        x, y, r = circles[0][0]
        x_p = int(round(x))
        print(x_p)
        for i in circles[0, :]:
            # 如果球很远，用绿色画出来
            if int(round(i[2])) < 30:
                cv2.circle(frame, (int(round(i[0])), int(round(i[1]))), int(round(i[2])), (0, 255, 0), 5)
                cv2.circle(frame, (int(round(i[0])), int(round(i[1]))), 2, (0, 255, 0), 10)
            # 或者用红色画
            elif int(round(i[2])) > 35:
                cv2.circle(frame, (int(round(i[0])), int(round(i[1]))), int(round(i[2])), (0, 0, 255), 5)
                cv2.circle(frame, (int(round(i[0])), int(round(i[1]))), 2, (0, 0, 255), 10)

    # 在帧中显示结果
    cv2.imshow('HueComp', hthresh)
    cv2.imshow('SatComp', sthresh)
    cv2.imshow('ValComp', vthresh)
    cv2.imshow('closing', closing)
    cv2.imshow('tracking', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
