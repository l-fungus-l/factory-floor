# Standard library imports
import logging
import math
import time
from time import perf_counter, sleep

# Third party imports
import cv2
import numpy as np
import paho.mqtt.client as mqtt
import serial

# Local application imports
import camera

# Constants
COLOR_CHECKER_X1 = 1559
COLOR_CHECKER_Y1 = 252
TARGET_COLOR1 = np.array([131, 70, 85])
COLOR_CHECKER_X2 = 183
COLOR_CHECKER_Y2 = 856
TARGET_COLOR2 = np.array([120, 60, 85])
CANNY_THRESHOLD_1 = 100
CANNY_THRESHOLD_2 = 200
HOUGH_CIRCLE_PARAM_1 = 100
HOUGH_CIRCLE_PARAM_2 = 100
MIN_CIRCLE_RADIUS = 0
MAX_CIRCLE_RADIUS = 0
NAME = "group22"

#detection stuff
can_detect = 1
check_white = 1
check_black = 1

workingCamera1 = True
workingCamera2 = True

whiteAngleDefault = 87
whiteAnglePush = 135

blackAngleDefault = 0
blackAnglePush = 45

LOWER_LIGHT_SENS_1 = 1
UPPER_LIGHT_SENS_1 = 4100

DEFAULT_LIGHT_SENS_2 = 0
LOWER_LIGHT_SENS_2 = 1
UPPER_LIGHT_SENS_2 = 4100

DEFAULT_LIGHT_SENS_3 = 0
LOWER_LIGHT_SENS_3 = 1
UPPER_LIGHT_SENS_3 = 4100


# Initialize variables for the phototransistors
catapult_servo_position = 3500
default_arm_phototransitor1 = 0
default_arm_phototransitor2 = 0

# Initialize the cameras
esp_cam = camera.Camera(1, 1233, 700, 1451, 932)

# Detection variable
can_detect = 1
check_white = 1
canIWork = None

# Set up serial connections
BLACK_WHITE_ARDUINO = serial.Serial('COM5', 9600)
BLACK_WHITE_ARDUINO.timeout = 0.1

FACTORY_PUSH_ARDUINO = serial.Serial('COM4', 9600)
FACTORY_PUSH_ARDUINO.timeout = 0.1

# Set up logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s %(levelname)s %(NAME)s %(message)s'
)


def on_connect(client, userdata, flags, rc):
    global NAME
    global reconnecting
    if not reconnecting:
        client.publish("robots/progress/{}".format(NAME), 0, qos=1)

def on_disconnect(client, userdata, rc):
    global NAME
    global reconnecting
    global canIWork
    global doNotReconnect
    canIWork = False
    reconnecting = True
    print("Disconnected with result code: {}".format(rc))
    reconnect_count, reconnect_delay = 0, 2
    while reconnect_count < 12 and not doNotReconnect:
        print("Reconnecting in {} seconds...".format(reconnect_delay))

        sleep(reconnect_delay)

        try:
            client.reconnect()
            print("Reconnected successfully!")
            client.publish("robots/progress/{}".format(NAME), 2, qos=1)
            client.subscribe("robots/allowed/{}".format(NAME), qos=1)
            return
        except Exception as err:
            print("{}. Reconnect failed. Retrying...".format(err))

        reconnect_delay *= 2
        reconnect_delay = min(reconnect_delay, 60)
        reconnect_count += 1
    print("Reconnect failed after {} attempts. Exiting...".format(reconnect_count))

def on_message(client, userdata, message):
    global canIWork
    global doNotReconnect
    if message.topic == "robots/allowed/{}".format(NAME):
        canIWork = int(message.payload.decode("utf-8"))
        if canIWork == 1:
            print("I can take the next disk.")
        else:
            print("I can NOT take the next disk.")
    else:
        canIWork = False
        print("Something is broken with me")
        doNotReconnect = True
        client.disconnect()


# MQTT setup
mqttBroker = "mqtt.eclipse.org"
client = mqtt.Client("Robot")
client.on_connect = on_connect
client.on_message = on_message
client.connect(mqttBroker)

# Function definitions
def check_cameras(esp_cam):
    _, image = esp_cam.cap.read()
    if (image is None):
        esp_cam.cap.release()
        print("Camera 1 not working, connecting to camera 2")
        error(201)

        esp_cam = camera.Camera(2, 308, 196, 537, 407)
        _, image = esp_cam.cap.read()
        print("Checking connection to camera 2")

        currentTime = perf_counter()
        while image is None and perf_counter() - currentTime < 60:
            _, image = esp_cam.cap.read()
            cv2.waitKey(1)
        if (image is None):
            print("None of the cameras are working :( ")
            error(301)

    if (not check_pixel_color(image, COLOR_CHECKER_X2, COLOR_CHECKER_Y2, TARGET_COLOR1) and workingCamera1 == False):
        print("None of the cameras are working :( ")
        error(311)

def check_pixel_color(image, x, y, target_color, tolerance=100):
    # Extract the color of the specified pixel
    pixel_color = image[y, x, :]

    # Calculate the difference between the pixel color and the target color
    color_diff = np.abs(pixel_color - target_color)

    # Check if the color difference is within the specified tolerance
    if np.all(color_diff <= tolerance):
        return True
    else:
        return False
# Function for detecting the color of the bumpers in certain positions

def compare_pixel_colors(default_value, color):
    # Calculate the Euclidean distance between two colors
    def color_distance(color1, color2):
        r1, g1, b1 = color1
        r2, g2, b2 = color2
        return np.sqrt((r2 - r1)**2 + (g2 - g1)**2 + (b2 - b1)**2)

    # Calculate the color distance
    distance = color_distance(color, default_value)

    # Define a threshold to determine the proximity
    threshold = 50

    # Check if the color distance is within the threshold
    result = 1 if distance < threshold else 0

    return result
# function to check if the cameras are working

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
            if circle.color[0] > 100:
                return "WHITE"
            else:
                return "BLACK"
    return "NONE"

def error(code):
    BLACK_WHITE_ARDUINO.write(f"E {code}\n".encode())

def getDefaultBumperColor(image):
    val1 = image[623, 570]
    val2 = image[637, 1000]
    return (val1, val2)
# get the default rgb values for the bumpers when moved

def getWhenMovedColor(image):
    val1 = image[657, 697]
    val2 = image[687, 1007]
    return (val1, val2)


def parseLightSensorData(data):
    """
    This function parses the data from the light sensor and returns the high of the sensor or returns an error
    """
    sensor = data.split(": ")[0]
    value = int(data.split(": ")[1].strip())
    if sensor == 'S1':
        if value < UPPER_LIGHT_SENS_1 and value > LOWER_LIGHT_SENS_1:
            return value
        else:
            return "ERROR"  
    if sensor == 'S2':
        if value < UPPER_LIGHT_SENS_2 and value > LOWER_LIGHT_SENS_2:
            return value
        else:
            return "ERROR"
    if sensor == 'S3':
        if value < UPPER_LIGHT_SENS_3 and value > LOWER_LIGHT_SENS_3:
            return value
        else:
            return "ERROR"

    return "ERROR"

    # ...

# Main function
if __name__ == "__main__":
    # Check the connection to the cameras
    print("Checking connection to camera 2")
    current_time = perf_counter()
    _, image = esp_cam.cap.read()
    while image is None and perf_counter() - current_time < 60:
        _, image = esp_cam.cap.read()
        cv2.waitKey(1)

    if image is None:
        print("None of the cameras are working :(")
        error(301)

    # Sleep in order for the Arduinos to do their setups
    sleep(2)

    # Put the servos in their default place
    print("Getting the servos to default position")
    BLACK_WHITE_ARDUINO.write(f"B {blackAngleDefault}\n".encode())
    sleep(0.01)
    BLACK_WHITE_ARDUINO.write(f"W {whiteAngleDefault}\n".encode())
    sleep(0.01)

    # Check the color of the bumpers in the lighting condition on the site
    print("Checking connection to the first camera")
    sleep(0.2)
    current_time = perf_counter()
    _, image = esp_cam.cap.read()
    while image is None and perf_counter() - current_time < 60:
        _, image = esp_cam.cap.read()
        cv2.waitKey(1)

    # Getting the default color of the bumpers
    print("Getting the default color of the bumpers")
    default_white_color, default_black_color = getDefaultBumperColor(image)

    # Move both of the servos
    BLACK_WHITE_ARDUINO.write(f"B {blackAnglePush}\n".encode())
    sleep(0.01)
    BLACK_WHITE_ARDUINO.write(f"W {whiteAnglePush}\n".encode())
    sleep(1)  # wait for the arms to move

    # Main loop
while True:
    # Check if the robot is allowed to work
    if canIWork:
        # Check if the robot can detect
        if can_detect:
            # Get the color of the circle currently on the belt
            circleColor = detected_circle(esp_cam)

            # If the circle color is white
            if circleColor == "WHITE":
                # If the robot is allowed to check white
                if check_white:
                    # Move the white servo
                    BLACK_WHITE_ARDUINO.write(f"W {whiteAnglePush}\n".encode())
                    sleep(0.01)
                    # Set check_white to 0
                    check_white = 0

            # If the circle color is black
            elif circleColor == "BLACK":
                # If the robot is allowed to check black
                if check_black:
                    # Move the black servo
                    BLACK_WHITE_ARDUINO.write(f"B {blackAnglePush}\n".encode())
                    sleep(0.01)
                    # Set check_black to 0
                    check_black = 0

            # If the circle color is none
            elif circleColor == "NONE":
                # Set check_white and check_black to 1
                check_white = 1
                check_black = 1

        # If the robot cannot detect
        else:
            # Set check_white and check_black to 1
            check_white = 1
            check_black = 1

        # Read data from the light sensor
        data = FACTORY_PUSH_ARDUINO.readline().decode().strip()

        # If data is not empty
        if data:
            # Parse the light sensor data
            sensor_value = parseLightSensorData(data)

            # If sensor value is not an error
            if sensor_value != "ERROR":
                # If sensor value is greater than default light sensor 2
                if sensor_value > DEFAULT_LIGHT_SENS_2:
                    # Move the white servo to default position
                    BLACK_WHITE_ARDUINO.write(f"W {whiteAngleDefault}\n".encode())
                    sleep(0.01)
                    # Set check_white to 1
                    check_white = 1

                # If sensor value is less than or equal to default light sensor 2
                else:
                    # Move the black servo to default position
                    BLACK_WHITE_ARDUINO.write(f"B {blackAngleDefault}\n".encode())
                    sleep(0.01)
                    # Set check_black to 1
                    check_black = 1

        # If data is empty
        else:
            # Set check_white and check_black to 1
            check_white = 1
            check_black = 1

    # If the robot is not allowed to work
    else:
        # Set check_white and check_black to 1
        check_white = 1
        check_black = 1

    # Sleep for a short duration
    sleep(0.01)

