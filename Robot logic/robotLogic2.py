# imports
import serial
import paho.mqtt.client as mqtt
from time import perf_counter
from time import sleep
from time import time
import numpy as np
import camera
import logging
import cv2
import math
from time import sleep
import paho.mqtt.client as mqtt

logging.basicConfig(level=logging.DEBUG, filename="robotLogic.log",
                    format='%(asctime)s %(levelname)s %(name)s %(message)s', filemode='w+')
# TO DO: get camera error detection by checking if the camera sees a pixel with a certain color

# Getting the serials done:
BlackWhiteArduino = serial.Serial(
    'COM5', 9600)
BlackWhiteArduino.timeout = 0.1
print("STARTUP")

FactoryPushArduino = serial.Serial('COM11', 9600)
FactoryPushArduino.timeout = 0.1
# CONSTANTS+
# pixel to look at for camera input testing:
colorCheckerX1 = 1014
colorCheckerY1 = 642
targetcolor1 = np.array([145, 125, 207])
colorCheckerX2 = 753
colorCheckerY2 = 465
targetcolor2 = np.array([120, 99, 211])

# Create booleans and values for the phototransistors
catapultServoPosition = 3500

defaultArmPhototransitor1 = 0
defaultArmPhototransitor2 = 0

# Initialize the cameras
#esp_cam = camera.Camera(1, 1233, 700, 1451, 932)
esp_cam = camera.Camera(2, 1233, 700, 1451, 932)

#detection stuff
can_detect = 1
check_white = 1
check_black = 1

workingCamera1 = True
workingCamera2 = True

whiteAngleDefault = 87
whiteAnglePush = 125

blackAngleDefault = 0
blackAnglePush = 45

lowerLightSens1 = 1
upperLightSens1 = 4100

defaultLightSens2 = 0
lowerLightSens2 = 1
upperLightSens2 = 4100

defaultLightSens3 = 0
lowerLightSens3 = 1
upperLightSens3 = 4100

# LOCAL VARIABLES

# Declare in which state the 2 bumpers should be
BlackPosition = 0
WhitePosition = 0

haveDetectedCatapult = False

# the time at which circle was pushed on the belt, none if there is no circle on the belt
whenPuckOnBelt = None

# create timers
timers = []

# Create phototransistors data
lightSens1Data = 0
lightSens2Data = 0
lightSens3Data = 0

# Is the current photo transistor value copied over from the default?
lightSens1WasTaken = False
lightSens2WasTaken = False
lightSens3WasTaken = False

lightSens1TriggerTime = 0

# The color of the circle currently on the beltA
circleColor = "NONE"


reconnecting = False
# set to True if you don't want to reconnect
doNotReconnect = False

#declare send error function
def error(code):
    BlackWhiteArduino.write(f"E {code}\n".encode())

# communication

canIWork = None
name = "group22"
reconnecting = False
# set to True if you don't want to reconnect
doNotReconnect = False



def on_connect(client, userdata, flags, rc):
    global name
    global reconnecting
    if not reconnecting:
        client.publish("robots/progress/{}".format(name), 0, qos=1)
    logging.debug("Connect to MQTT broker")


def on_disconnect(client, userdata, rc):
    global name
    global reconnecting
    global canIWork
    global doNotReconnect
    canIWork = False
    reconnecting = True
    logging.debug("Disconnected with result code: {}".format(rc))
    reconnect_count, reconnect_delay = 0, 2
    while reconnect_count < 12 and not doNotReconnect:
        logging.debug("Reconnecting in {} seconds...".format(reconnect_delay))

        sleep(reconnect_delay)

        try:
            client.reconnect()
            logging.debug("Reconnected successfully!")
            client.publish("robots/progress/{}".format(name), 2, qos=1)
            client.subscribe("robots/allowed/{}".format(name), qos=1)
            return
        except Exception as err:
            logging.debug("{}. Reconnect failed. Retrying...".format(err))

        reconnect_delay *= 2
        reconnect_delay = min(reconnect_delay, 60)
        reconnect_count += 1
    logging.debug("Reconnect failed after {} attempts. Exiting...".format(reconnect_count))
    error(111)
# either from the allowed topic or from the error topic about a fault

def on_message(client, userdata, message):
    global canIWork
    global doNotReconnect
    if message.topic == "robots/allowed/{}".format(name):
        canIWork = int(message.payload.decode("utf-8"))
        if canIWork == 1:
            logging.debug("I can take the next disk.")
        else:
            logging.debug("I can NOT take the next disk.")
    else:
        canIWork = False
        if message.payload.decode("utf-8") == "processing at high rate" :
            error(111)
            logging.error("Something is broken with me")
        else :
            error(111)
            logging.error("Communication error")
        doNotReconnect = True
        client.disconnect()


mqttBroker = '172.26.0.108'
client = mqtt.Client(name)

client.on_connect = on_connect
client.on_disconnect = on_disconnect
# set LWT: publish name in the 'disconnected' topic
client.will_set("disconnected", payload=name, qos=1, retain=False)

try:
    client.connect(mqttBroker, keepalive=10)
except Exception as err:
    logging.debug(err)
client.subscribe("robots/allowed/{}".format(name), qos=1)
client.subscribe("robots/errors/{}".format(name), qos=1)

logging.debug("Communication started")

mqttBroker = '172.26.0.108'
client.connect(mqttBroker)
client.subscribe(f"robots/allowed/group22", qos=1)

client.loop_start()
client.on_message = on_message
# CV2 uglyness
# Create a new namded window for the controls
cv2.namedWindow("control")
cv2.namedWindow("preview")

cv2.createTrackbar('CANNY_THRESHOLD_1', 'control',
                   camera.CANNY_THRESHOLD_1, 255, lambda x: x)
cv2.createTrackbar('CANNY_THRESHOLD_2', 'control',
                   camera.CANNY_THRESHOLD_2, 255, lambda x: x)
cv2.createTrackbar('PARAM1', 'control',
                   camera.HOUGH_CIRCLE_PARAM_1, 255, lambda x: x)
cv2.createTrackbar('PARAM2', 'control',
                   camera.HOUGH_CIRCLE_PARAM_2, 255, lambda x: x)
cv2.createTrackbar('MIN_CIRCLE_RADIUS', 'control',
                   camera.MIN_CIRCLE_RADIUS, 255, lambda x: x)
cv2.createTrackbar('MAX_CIRCLE_RADIUS', 'control',
                   camera.MAX_CIRCLE_RADIUS, 255, lambda x: x)

# The following function is meant to detect whether the bumpers are in the correct place by checking a pixel
# on the camera and whether the pixel is close to the yellow of the camera


def compare_pixel_colors(color1, color2):
    # Calculate the Euclidean distance between two colors

    # Calculate the color distance
    
    r1, g1, b1 = color1
    r2, g2, b2 = color2
    
    threshold = 40
    # Define a threshold to determine the proximity
    if((abs(int(r1)-int(r2)) < threshold) and (abs(int(b1)-int(b2)) < threshold)and (abs(int(g1)-int(g2)) < threshold)):
        result = True
    else:
        result = False

    return result
# function to check if the cameras are working


def check_pixel_color(image, x, y, target_color, tolerance=50):
    # Extract the color of the specified pixel


    r1, g1, b1 = image[y,x]
    r2,g2,b2 = target_color
    if(abs(int(r1)-int(r2)) + abs(int(g1)-int(g2)) + abs(int(b1)-int(b2)) < tolerance):
        return True
    else:
        return False
    # Check if the color difference is within the specified tolerance
# Function for detecting the color of the bumpers in certain positions
# get the default rgb values for the bumpers


def getDefaultBumperColor(image):
    if(esp_cam.url == 1):
        val1 = image[423, 1243]
        val2 = image[444, 915]
    else:
        val1 = image[687, 531]
        val2 = image[675, 853]
    return (val1, val2)
# get the default rgb values for the bumpers when moved

def getWhenMovedColor(image):
    if(esp_cam.url == 1):
        val1 = image[387, 1100]
        val2 = image[372, 778]
    else:
        val1 = image[720, 589]
        val2 = image[708, 937]
    return (val1, val2)


def parseLightSensorData(data):
    """
    This function parses the data from the light sensor and returns the high of the sensor or returns an error
    """
    sensor = data.split(": ")[0]
    value = int(data.split(": ")[1].strip())
    if sensor == 'S1':
        if value < upperLightSens1 and value > lowerLightSens1:
            return value
        else:
            return "ERROR"  
    if sensor == 'S2':
        if value < upperLightSens2 and value > lowerLightSens2:
            return value
        else:
            return "ERROR"
    if sensor == 'S3':
        if value < upperLightSens3 and value > lowerLightSens3:
            return value
        else:
            return "ERROR"

    return "ERROR"


def detected_circle(esp_cam):
    """
    This function detects the color of the current puck under the camera. It also previews the output in the CV2 named window preview.
    returns: WHITE, BLACK or NONE. With none beeing no circle detected. 
    """
    camera.CANNY_THRESHOLD_1 = cv2.getTrackbarPos(
        'CANNY_THRESHOLD_1', 'control')
    camera.CANNY_THRESHOLD_2 = cv2.getTrackbarPos(
        'CANNY_THRESHOLD_2', 'control')
    camera.HOUGH_CIRCLE_PARAM_1 = cv2.getTrackbarPos('PARAM1', 'control')
    camera.HOUGH_CIRCLE_PARAM_2 = cv2.getTrackbarPos('PARAM2', 'control')
    camera.MIN_CIRCLE_RADIUS = cv2.getTrackbarPos(
        'MIN_CIRCLE_RADIUS', 'control')
    camera.MAX_CIRCLE_RADIUS = cv2.getTrackbarPos(
        'MAX_CIRCLE_RADIUS', 'control')

    circles = esp_cam.get_circles()
    cv2.imshow("preview", esp_cam.previous_frame.preview())

    if len(circles) > 1:
        logging.error("More than one circle detected")
        return "ERROR"
    elif len(circles) == 1:
        # check if the circle.color trouple is grayscale
        circle = circles[0]

        if math.isclose(circle.color[0], circle.color[1], abs_tol=30) and math.isclose(circle.color[0], circle.color[2], abs_tol=30):
            # check if the cirle is white or black
            if circle.color[0] > 150:
                return "WHITE"
            else:
                return "BLACK"
    return "NONE"

def check_cameras(esp_cam):
    _, image = esp_cam.cap.read()
    if (image is None):
        if(esp_cam.url  == 1):
            currentTime = perf_counter()
            while image is None and perf_counter() - currentTime < 60:
                _, image = esp_cam.cap.read()
                cv2.waitKey(1)
            if (image is None):
                logging.debug("Camera 1 not connected ")
                error(201)
            elif not check_pixel_color(image, colorCheckerX1, colorCheckerY1, targetcolor1):
                logging.debug("Camera 1 self check failed")
                error(211)
        else:
            currentTime = perf_counter()
            while image is None and perf_counter() - currentTime < 60:
                _, image = esp_cam.cap.read()
                cv2.waitKey(1)
            if (image is None):
                logging.debug("Camera 1 not connected ")
                error(201)
            elif not check_pixel_color(image, colorCheckerX1, colorCheckerY1, targetcolor1):
                logging.debug("Camera 1 self check failed")
                error(211)
    elif (esp_cam.url == 1 and not check_pixel_color(image, colorCheckerX1, colorCheckerY1, targetcolor1)):
        logging.debug("Camera 1 failed self check")
        error(211)
    elif (esp_cam.url == 2 and not check_pixel_color(image, colorCheckerX2, colorCheckerY2, targetcolor2)):
        logging.debug("Camera 2 failed sefl check")
        error(212)

sleep(4)
check_cameras(esp_cam)
# sleep in order for the arduinos to do there setups
sleep(2)

print("Getting the servos to default position")

# Put the servos in their default place
BlackWhiteArduino.write(f"B {blackAngleDefault}\n".encode())
sleep(0.01)
BlackWhiteArduino.write(f"W {whiteAngleDefault}\n".encode())
sleep(0.01)

# Check the color of the bumpers in the lighting condition on the site
print("Checking connection to the first camera")
sleep(0.2)
currentTime = perf_counter()
_, image = esp_cam.cap.read()
while image is None and perf_counter() - currentTime < 60:
    _, image = esp_cam.cap.read()
    cv2.waitKey(1)

# getting the default color of the bumpers for the lighting condition on the site
logging.debug("Getting the default color of the bumpers")
defaultWhiteColor, defaultBlackColor = getDefaultBumperColor(image)

# Move both of the servos
BlackWhiteArduino.write(f"B {blackAnglePush}\n".encode())
sleep(0.01)
BlackWhiteArduino.write(f"W {whiteAnglePush}\n".encode())
sleep(1) # wait for the arms to move

# Getting the when moved color of the bumpers for the lighting condition on the site
logging.debug("Getting the when moved color of the bumpers")
whenMovedWhiteColor, whenMovedBlackColor = getWhenMovedColor(esp_cam.cap.read()[1])

BlackWhiteArduino.write(f"B {blackAngleDefault}\n".encode())
BlackWhiteArduino.write("R0\n".encode())
sleep(0.01)
BlackWhiteArduino.write(f"W {whiteAngleDefault}\n".encode())
FactoryPushArduino.write(f"C1\n".encode())
sleep(1) # wait for the arms to move

while True:
    # get current time in seconds 
    currentTime = perf_counter()
    # Read the input of the phototransistor
    logging.debug("Reading the input of the phototransistors")
    try:
        a = FactoryPushArduino.readline()
        factoryData = a.decode().strip()
        parts = factoryData.split()

    except UnicodeDecodeError:
        factoryData = None

    # FactoryPushArduino.reset_input_buffer()

    # Check if there's something in factoryData
    if factoryData is not None and factoryData != "":
        # Get the parts from factoryData
        # Now we check for what kind of data is in factoryData
        # Check if there is the value of the phototransistor
        if 'L' in factoryData:
            client.publish(f"robots/progress/group22", 1, qos=1)
            

    
    check_cameras(esp_cam)

    image = esp_cam.cap.read()[1]
    if (image is not None):
        logging.debug("Checking if the bumpers are in the correct position")
        if(check_black == 1):
            if (BlackPosition == 0):
                if(esp_cam.url == 1):
                    color = image[444, 915]
                else :color = image[675, 853]
                if (not compare_pixel_colors(defaultBlackColor, color)):
                    # send error message
                    logging.debug("ERROR: Black bumper not in correct position")
                    error(301)

            elif (BlackPosition == 1):
                if(esp_cam.url == 1):image[372, 778] 
                else: color = image [708, 937]
                if (not compare_pixel_colors(whenMovedBlackColor, color)):
                    # send error message
                    logging.debug("ERROR: Black bumper not in correct position")
                    error(301)
        if(check_white == 1):
            if (WhitePosition == 0):
                if(esp_cam.url == 1):
                    color = image[423, 1243]
                else: color = image[687,531]
                if (not compare_pixel_colors(defaultWhiteColor, color)):
                    # send error message
                    logging.debug("ERROR: White bumper not in correct position")
                    error(302)

            elif (WhitePosition == 1):
                if(esp_cam.url == 2):
                    color = image [720, 589]
                else: color = image[387, 1100]
                if (not compare_pixel_colors(whenMovedWhiteColor, color)):
                    # send error message
                    logging.debug("ERROR: White bumper not in correct position")
                    error(302)
    # Puck pushing from Factory floor:

    # Circle detection
    # Check if circles can be detected
    logging.debug("Starting circle detection")
    if (can_detect == 1):
        a = detected_circle(esp_cam)
        if not a == "NONE":
            circleColor = a
            logging.debug("Not black or white circle detected")
        else:
            circleColor = "No circle or circle that is not black or white detected"
            # Push the puck from Circle detection

            # Insert the color of the puck in the Arduino, as well as the angle for the servos
            # Move the servos with white/black angle value
            # Put the timers in the Timers array, as well as the stuff to tell to the pie
        if (circleColor == 'WHITE'):
            # Can no longer detect circles
            can_detect = 0
            # Print the detected circle
            logging.debug("White circle detected")
            # Make the White servo move
            logging.debug("Move White servo to pushing position")
            timers.append([1, currentTime, f"W {whiteAnglePush}\n"])
            # Tell the white servo to move to default after 9 seconds
            timers.append([2, currentTime+26, 'W0'])
            # update the white position
            WhitePosition = 1
            # sleep to wait for the servo to move
            check_white = 0
            timers.append([2, currentTime+1,'W1'])

        elif (circleColor == "BLACK"):
            can_detect = 0
            logging.debug("Black Circle detected")
            logging.debug("Move Black servo to pushing position")

            timers.append([1, currentTime, f"B {blackAnglePush}\n"])

            timers.append([1, currentTime+20, f"B {blackAngleDefault}\n"])
            # update the black position
            BlackPosition = 1
            # sleep to wait for the servo to move
            check_black = 0
            timers.append([2, currentTime+1,'B'])

        elif circleColor == "NONE":
           logging.debug("MultiColor circle detected")

        # Read the data from the BlackWhiteArduino
        logging.debug("Checking the data from the catapult phototransistor")
        bw_arduino_data = BlackWhiteArduino.readline().decode()
        BlackWhiteArduino.reset_input_buffer()

        # if bw_arduino_data.split(': ')[0] == 'S3':
        #     lightSens3Data = parseLightSensorData(bw_arduino_data)
        #     logging.debug(f'lightSens3Data: {lightSens3Data}')


        #     # Set the reference photoresolution for the catapult, if it hasn't been set yet
        #     if (lightSens3WasTaken == False and lightSens3Data > 0 and lightSens3Data < 1200):
        #         logging.debug("Setting the reference photoresolution for the catapult")
        #         lightSens3WasTaken = True
        #         defaultLightSens3 = lightSens3Data
        #         logging.debug(f"defaultLightSens3: {defaultLightSens3}")
        #     elif abs(lightSens3Data-defaultLightSens3) > 200 and not any("R 90\n" in s for s in timers):
        #         # Check if "R 90" is already in the timers array
        #         logging.debug("Starting catapult")
        #         BlackWhiteArduino.write("R 90 \n".encode())
        #         BlackWhiteArduino.write(f"S {catapultServoPosition} \n".encode())


        #         timers.append([1, currentTime+12.5, f"S 0\n"])
        #         timers.append([1, currentTime+11, "R 0\n"])
        if bw_arduino_data.split(': ')[0] == 'triggered_slingshot':
            can_detect = 1
            logging.debug("Catapult has been triggered")
            FactoryPushArduino.write("C1\n".encode())
        # Check if anything time related needs to end

    for index, timer in enumerate(timers):
        # if the current time is higher than the timer
        if (currentTime > float(timer[1])):
            # check for which arduino it is
            # if the timer is for the Factory arduino
            if (timer[0] == 0):
                #logging.debug( in the Arduino the default angle for the servo
                FactoryPushArduino.write(timer[2].encode())
                timers.pop(index)
            # else if the timer is for the BlackWhite arduino, thenlogging.debug( and pop what is in timer
            if (timer[0] == 1):
                if 'W' in timer[2]:
                    logging.debug("Moving White servo to default position")
                    BlackWhiteArduino.write(timer[2].encode())
                    timers.pop(index)

                    if timer[2] == "W {whiteAngleDefault}\n":
                        can_detect = 1
                    
                    WhitePosition = 0
                    # sleep in order for the servo to get back in position
                    check_white = 0
                    timers.append([2, currentTime+1,'W1'])
                    FactoryPushArduino.write("C1\n".encode())
                elif 'B' in timer[2]:
                    logging.debug("Moving Black servo to default position")
                    BlackWhiteArduino.write(timer[2].encode())
                    timers.pop(index)

                    if timer[2] == "B {blackAngleDefault}\n":
                        can_detect = 1
                    
                    BlackPosition = 0
                    # sleep in order for the servo to get back in position
                    check_black = 0
                    timers.append([2, currentTime+1,'B1'])
                    FactoryPushArduino.write("C1\n".encode())
                elif 'S' in timer[2]:
                    logging.debug("Moving catapult servo to default position")
                    BlackWhiteArduino.write(timer[2].encode())
                    timers.pop(index)

                    if timer[2] == "S 0\n":
                        can_detect = 1

                    haveDetectedCatapult = False
                elif 'R' in timer[2]:
                    logging.debug("Launching the catapult")
                    BlackWhiteArduino.write(timer[2].encode())
                    timers.pop(index)

                    can_detect = 1
                    
                    FactoryPushArduino.write("C1\n".encode())
            if(timer[0] == 2):
                if(timer[2] == 'W1'):
                    check_white = 1
                    whiteServoPosition = 1
                elif(timer[2] == 'W0'):
                    check_white =1
                    whiteServoPosition = 0
                elif(timer[2] == 'B1'):
                    check_black = 1
                    blackServoPosition = 1
                elif(timer[2] == 'B0'):
                    check_black = 1
                    blackServoPosition = 0
                    
    cv2.waitKey(1)