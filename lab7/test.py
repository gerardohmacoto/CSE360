# this code works only on the raspberry
import numpy as np
import cv2 as cv
# from picamera2 import Picamera2
import sys
# from Motor import *
import math
import time

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# picam2 = Picamera2()
# picam2.configure(picam2.create_preview_configuration(
#     main={"format": 'XRGB8888', "size": (640, 480)}))
# picam2.start()


while True:
    try:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # 640x480
        # width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
        # height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
        # print(width, height)
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        lower = np.array([20, 80, 80])
        upper = np.array([30, 255, 255])

        mask = cv.inRange(hsv, lower, upper)

        result = cv.bitwise_and(frame, frame, mask=mask)

        # using the mask, draw a boundary of the detected object
        # contours, hierarchy = cv.findContours(mask.copy(),cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # output = cv.drawContours(result,contours, -1, (0,0,255),3)

        # result = cv.bitwise_and(frame, frame, mask = mask)

        display = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
        # show_test = cv.medianBlur(hsv_test, 5)

        # Setup SimpleBlobDetector parameters.
        params = cv.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 25
        params.maxThreshold = 700

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 75
        params.maxArea = 20000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.05
        # Create a detector with the parameters
        ver = (cv.__version__).split('.')
        if int(ver[0]) < 3:
            detector = cv.SimpleBlobDetector(params)
        else:
            detector = cv.SimpleBlobDetector_create(params)
        # Detect blobs.
        keypoints = detector.detect(result)
        blobCount = len(keypoints)
        # Draw detected blobs as green circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        # im_with_keypoints = cv.drawKeypoints(result, keypoints, np.array(
        #     []), (255, 255, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Our operations on the frame come here
        # gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # Display the resulting frame
        font = cv.FONT_HERSHEY_SIMPLEX
        text = "Count=" + str(blobCount)

        cv.putText(display, text, (5, 25), font, 1, (0, 255, 0), 2)

        # -------------------------

        if blobCount > 0:
            blob_x = keypoints[0].pt[0]
            text2 = "X="+"{:.2f}".format(blob_x)
            cv.putText(display, text2,
                       (5, 50), font, 1, (0, 255, 0), 2)

            blob_y = keypoints[0].pt[1]
            text3 = "Y=" + "{:.2f}".format(blob_y)
            cv.putText(display, text3,
                       (5, 75), font, 1, (0, 255, 0), 2)
            # Write Size of first blob
            blob_size = keypoints[0].size
            text4 = "S=" + "{:.2f}".format(blob_size)
            cv.putText(display, text4,
                       (5, 100), font, 1, (0, 255, 0), 2)
            cv.circle(display, (int(blob_x), int(blob_y)),
                      int(blob_size / 2), (0, 0, 255), 5)
    # -------------------------

        cv.imshow('keypoints', display)

        if cv.waitKey(1) == ord('q'):
            break
    except KeyboardInterrupt:
        #         for i in range(1):
        #             imgName = "opencv_frame_{}.png".format(i+10)
        #             cv.imwrite(imgName, result)
        exit()


# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
