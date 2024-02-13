import cv2
import numpy as np

ESP_CAM_1 = "http://192.168.8.3/stream"
# ESP_CAM_2 = "http://192.168.8.4/stream"

MAX_SPEED = 10
MAX_RADIUS_CHANGE = 3

MAX_GONE_TIME = 0
MIN_HERE_TIME = 0

CANNY_THRESHOLD_1 = 31
CANNY_THRESHOLD_2 = 28
HOUGH_CIRCLE_PARAM_1 = 2
HOUGH_CIRCLE_PARAM_2 = 29
MIN_CIRCLE_RADIUS = 63
MAX_CIRCLE_RADIUS = 82

class Camera:
    def __init__(self, url, x1, y1, x2, y2) -> None:
        self.cap = cv2.VideoCapture(url)
        self.url = url
        self.previous_frame = None

        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

        self.get_circles()
        
    def get_circles(self):
        #Get the high resolution video of the camera (default is 640x480)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        _, frame = self.cap.read()
        while frame is None:
            _, frame = self.cap.read()
            cv2.waitKey(1)
        #reframe the camera
        frame = frame[self.y1:self.y2,self.x1:self.x2]
        # frame = frame[640:870,1220:1480]

        # if not self.self_test(frame):
        #     return "ERROR"
        
        self.previous_frame = Frame(frame, self.previous_frame)

        confident_circles = []
        for circle in self.previous_frame.circles:
            if circle.f_here >= MIN_HERE_TIME:
                confident_circles.append(circle)

        return confident_circles
    
    def self_test(self, frame):
        # get the color of the blue square
        roi = frame.cv_frame[0:100, 0:100]
        b, g, r = cv2.split(roi)

        if b < 100 or g > 100 or r > 100:
            return False
        
        # get the color of the green square
        roi = frame.cv_frame[100:200, 0:100]
        b, g, r = cv2.split(roi)

        if b > 100 or g < 100 or r > 100:
            return False
        
        # get the color of the red square
        roi = frame.cv_frame[200:300, 0:100]
        b, g, r = cv2.split(roi)

        if b > 100 or g > 100 or r < 100:
            return False
        
        return True
        
    
def detect_circles_in_frame(frame):
    img = frame.copy()
    #crop image to specified area
    #img = img[y1:y2, x1:x2]
    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blur = cv2.GaussianBlur(gray, (9, 9), 0)

    # Detect edges using Canny edge detection
    edges = cv2.Canny(blur, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2)
    cv2.imshow('edges', edges)

    # Detect circles using Hough circle detection
    circles = cv2.HoughCircles(
        edges, cv2.HOUGH_GRADIENT, 1, 20, param1=HOUGH_CIRCLE_PARAM_1, param2=HOUGH_CIRCLE_PARAM_2, minRadius=MIN_CIRCLE_RADIUS, maxRadius=MAX_CIRCLE_RADIUS)

    # Draw detected circles on the original image
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(img, (x, y), r, (0, 255, 0), 2)

    return circles if circles is not None else [] 

class Circle:
    def __init__(self, x, y, r, frame):
        self.x = x
        self.y = y
        self.r = r

        self.f_gone = 0
        self.f_here = 0

        # Get the average color of the circle
        roi = frame.cv_frame[max(0, int(y-r)):max(0, int(y+r)), max(0, int(x-r)):max(0, int(x+r))]
        mask = np.zeros_like(roi)
        cv2.circle(mask, (int(r), int(r)), int(r), (255, 255, 255), -1)
        masked_roi = cv2.bitwise_and(roi, mask)
        b, g, r = cv2.split(masked_roi)
        self.color = (np.mean(b), np.mean(g), np.mean(r))

    def same_as(self, circle):
        return abs(self.x - circle.x) <= MAX_SPEED+MAX_SPEED*circle.f_gone and abs(self.y - circle.y) <= MAX_SPEED+MAX_SPEED*circle.f_gone and abs(self.r - circle.r) <= MAX_RADIUS_CHANGE+MAX_RADIUS_CHANGE*circle.f_gone



class Frame:
    def __init__(self, cv_frame, previous_frame=None):
        # Detect all the circles in the frame
        np_circles = detect_circles_in_frame(cv_frame)
        self.cv_frame = cv_frame

        # Convert all the numpy circles to circle objects
        self.circles = []
        for np_circle in np_circles:
            self.circles.append(
                Circle(np_circle[0], np_circle[1], np_circle[2], self))

        # If there is no previous frame we can not do object tracking
        if previous_frame is None:
            return

        # # Check if there are circles in this frame which where also in the previous frame
        # # If so increase the here of the circle
        # for circle in self.circles:
        #     for old_circle in previous_frame.circles:
        #         if circle.same_as(old_circle):
        #             circle.f_here = old_circle.f_here + 1
        #             circle.f_gone = 0
        #             break
        
        # # Check if there are images in the previous frame with a high confidence which disappeared
        # for old_circle in previous_frame.circles:
        #     disappreared = True
        #     for circle in self.circles:
        #         if circle.same_as(old_circle):
        #             disappreared = False
            
        #     if disappreared and old_circle.f_here >= MIN_HERE_TIME and old_circle.f_gone <= MAX_GONE_TIME:
        #         old_circle.f_gone += 1
        #         self.circles.append(old_circle)
        
    def preview(self):
        print('making preview')
        frame = self.cv_frame.copy()

        for circle in self.circles:
            # put the circle.f_here as text in the middle of the circle
            cv2.putText(frame, str(circle.f_here), (circle.x, circle.y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            # add f_gone as text in the top left corner of the circle
            cv2.putText(frame, str(circle.f_gone), (circle.x-circle.r, circle.y-circle.r), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

            if circle.f_here >= MIN_HERE_TIME:
                cv2.circle(frame, (circle.x, circle.y), circle.r, (0, 255, 0), 2)
            else:
                True
                cv2.circle(frame, (circle.x, circle.y), circle.r, (255, 0, 0), 2)

        
        return frame

if __name__ == "__main__":
    # open video
    video = cv2.VideoCapture('Robot logic/circles.mp4')

    # create window
    cv2.namedWindow("control")

    while True:
        ret, frame = video.read()

        if ret:
            if not 'prev_frame' in locals():
                prev_frame = Frame(frame)
            
            prev_frame = Frame(frame, prev_frame)

            cv2.imshow("preview", prev_frame.preview())

        else:
            break

        key = cv2.waitKey(500)