import numpy as np
import cv2
import camera

cap = cv2.VideoCapture(1)
#make the cap to be full hd
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
#make a slider for the canny thresholds and HoughCircle parameters
cv2.namedWindow("Trackbars")
cv2.createTrackbar("Canny Threshold 1", "Trackbars", camera.CANNY_THRESHOLD_1, 255, lambda x: None)
cv2.createTrackbar("Canny Threshold 2", "Trackbars", camera.CANNY_THRESHOLD_2, 255, lambda x: None)
cv2.createTrackbar("Hough Circle Threshold", "Trackbars", 1, 255, lambda x: None)
cv2.createTrackbar("Hough Circle Min Distance", "Trackbars", camera.MIN_CIRCLE_RADIUS , 255, lambda x: None)
cv2.createTrackbar("Hough Circle Param 1", "Trackbars", camera.HOUGH_CIRCLE_PARAM_1, 255, lambda x: None)
cv2.createTrackbar("Hough Circle Param 2", "Trackbars", camera.HOUGH_CIRCLE_PARAM_2, 255, lambda x: None)
cv2.createTrackbar("Hough Circle Min Radius", "Trackbars", camera.MIN_CIRCLE_RADIUS, 255, lambda x: None)
cv2.createTrackbar("Hough Circle Max Radius", "Trackbars", camera.MAX_CIRCLE_RADIUS, 255, lambda x: None)

#detect circles like in camera.py
while True:
    _,frame = cap.read()
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Blur the frame
    gray = cv2.medianBlur(gray, 5)
    # Detect the circles in the frame
    canny = cv2.Canny(gray, cv2.getTrackbarPos("Canny Threshold 1", "Trackbars"), cv2.getTrackbarPos("Canny Threshold 2", "Trackbars"))
    cv2.imshow("Canny", canny)
    circles = cv2.HoughCircles(canny, cv2.HOUGH_GRADIENT, 1, cv2.getTrackbarPos("Hough Circle Min Distance", "Trackbars"), param1=cv2.getTrackbarPos("Hough Circle Param 1", "Trackbars"), param2=cv2.getTrackbarPos("Hough Circle Param 2", "Trackbars"), minRadius=cv2.getTrackbarPos("Hough Circle Min Radius", "Trackbars"), maxRadius=cv2.getTrackbarPos("Hough Circle Max Radius", "Trackbars"))
    #draw the circles on the frame
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
    #show the frame
    cv2.imshow("Frame", frame)
    cv2.waitKey(1)
