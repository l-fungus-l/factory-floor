# imports AAAAA
import serial
import paho.mqtt.client as mqtt
from time import perf_counter
import camera
import logging
import cv2
import math
from time import sleep
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(levelname)s %(name)s %(message)s')

# TO DO: implement failure detection with timers for phototransistors
# TO DO: if I'm waiting too much time for data from the arduinos, then give an error
# TO DO: make changes to make a catapult instead of a slingshot
# TO DO: check if on the factory floor part of the robot the phototransistors have realistic values

# Getting the constants done

TIMER_ADDER = 5

# COMMUNICATION: connect the pi to the broker at startup

# canIWork : int = None # global variable to check if the robot can process the next disk (1 or 0)
# def on_connect(client, userdata, flags, rc, properties=None):
#     global name
#     client.publish(f"robots/progress/{name}", 0, qos=1)

# def on_disconnect(client, userdata, rc):
#     global name
#     #client.publish("disconnected", name, qos=1)
#     #client.loop_stop()
#     client.reconnect()

# def on_message(client, userdata, message):
#     global canIWork
#     canIWork = int(message.payload.decode("utf-8"))

# client = mqtt.Client("group22")

# client.on_connect = on_connect
# client.on_disconnect = on_disconnect
# # set LWT: publish name in the 'disconnected' topic
# client.will_set(f"disconnected", payload="group22", qos=1, retain=False)

# mqttBroker = '172.26.0.108'
# client.connect(mqttBroker)
# client.subscribe(f"robots/allowed/group22", qos=1)

# client.loop_start()
# client.on_message = on_message


# Getting the serials done:
BlackWhiteArduino = serial.Serial(
    'COM5', 9600)
# FactoryPushArduino = serial.Serial('/dev/ttyACMO', 9600)
BlackWhiteArduino.timeout = 0.1
# FactoryPushArduino.timeout = 0.1

# CONSTANTS
# Create booleans and values for the phototransistors
defaultCatapultPhototransistor = 0

defaultArmPhototransitor1 = 0
defaultArmPhototransitor2 = 0

esp_cam_1 = camera.Camera(1)
# esp_cam_2 = camera.Camera(1)

whiteAngleDefault = 0
whiteAnglePush = 45

blackAngleDefault = 90
blackAnglePush = 140


# LOCAL VARIABLES

# the time at which circle was pushed on the belt, none if there is no circle on the belt
whenPuckOnBelt = None

# create timers
timers = []

# Create phototransistors data
catapultPhotoData = 0
armPhotoData1 = 0
armPhotoData2 = 0

# Is the current photo transistor value copied over from the default?
wasTakenArmPhototransistor1 = False
wasTakenArmPhototransistor2 = False
wasTakenCatapultPhoto = False

# The color of the circle currently on the belt
circleColor = "NONE"

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


# sleep in order for the arduinos to do there setups
sleep(2)

#Put the servos in their default place
BlackWhiteArduino.write("B 90\n".encode())
BlackWhiteArduino.write("W 0\n".encode())

sleep(0.2)

whenPuckOnBelt = perf_counter()

while True:
    # get current time in seconds
    currentTime = perf_counter()

    # Puck pushing from Factory floor:

    # # Read the input of the phototransistor
    # factoryData = FactoryPushArduino.readline().decode().strip()
    # factoryData.flushInput()
    # # Check if there's something in factoryData
    # if factoryData is not None:

    #     # Get the parts from factoryData
    #     parts = factoryData.split()

    #     # Now we check for what kind of data is in factoryData
    #     # Check if there is the value of the phototransistor
    #     if parts is not None:

    #         if 'S1' in parts[0]:

    #             # if so, then put the value in lightSens
    #             lightSens1 = int(parts[1])

    #             # Do something with the light sensor value
    #             if(lightSens1 > val):

    #                 # we set the angle for the servo
    #                 angleFactory = 20

    #                 # message to tell to Arduino:
    #                 message = f"P {angleFactory} \n"
    #                 # write the message to Arduino
    #                 FactoryPushArduino.write(message)
    #                 # put the timer into the timers array
    #                 Timers.append([0, currentTime+1, "P 0\n"])
    #         if 'S2' in parts[2]:
    #             lightSens2 = int(parts[3])

    #             if lightSens2 > val :

    # Circle detection

    a = detected_circle(esp_cam_1)
    if not a == "NONE":
        circleColor = a

    # Push the puck from Circle detection

    # Insert the color of the puck in the Arduino, as well as the angle for the servos
    # Move the servos with white/black angle value
    # Put the timers in the Timers array, as well as the stuff to tell to the pie
    if (circleColor == 'WHITE'):
        print("White circle detected")
        BlackWhiteArduino.write(f"W {whiteAnglePush}\n".encode())
        timers.append([1, currentTime+9, f"W {whiteAngleDefault}\n"])
        # say that you have processed a disk
        # client.publish(f"robots/progress/group22", 1, qos=1)

    elif (circleColor == "BLACK"):
        print("Black Circle detected")
        BlackWhiteArduino.write(f"B {blackAnglePush}\n".encode())
        timers.append([1, currentTime+7, f"W {blackAngleDefault}\n"])
        # say that you have processed a disk
        # client.publish(f"robots/progress/group22", 1, qos=1)

    elif circleColor == "NONE":
        print("MultiColo circle detected")

    # Read tbe phototransistor data from the BlackWhiteArduino
    bw_arduino_data = BlackWhiteArduino.readline().decode()
    BlackWhiteArduino.flushInput()
    if bw_arduino_data.split(': ')[0] == 'S3':
        catapultPhotoData = int(bw_arduino_data.split(': ')[1].strip())
    if (wasTakenCatapultPhoto == False):
        wasTakenCatapultPhoto = True
        defaultCatapultPhototransistor = catapultPhotoData
    elif abs(catapultPhotoData-defaultCatapultPhototransistor) > 100:
        position = 3500
        BlackWhiteArduino.write("R 180 \n".encode())
        BlackWhiteArduino.write(f"S {position} \n".encode())
        timers.append([1, currentTime+10.2, f"S 0\n"])
        timers.append([1, currentTime+9.2, "R 90\n"])
        # say that you have processed a disk (catapult)
        # client.publish(f"robots/progress/group22", 1, qos=1)

    # timer[0] = which type of action to do
    # timer[1] = timer after which the action has to be done
    # timer[2] = message to send to arduino

# Check if anything time related needs to end

    done_times_indexes = []
    for index, timer in enumerate(timers):
        # if the current time is higher than the timer
        if (currentTime > timer[1]):
            # check for which arduino it is
            # if the timer is for the Factory arduino
            # if(timer[0]==0):
            #     #Print in the Arduino the default angle for the servo
            #     FactoryPushArduino.write(timer[2])
            #     Timers.pop(index)
            # else if the timer is for the BlackWhite arduino, then print and pop what is in timer
            if (timer[0] == 1):
                if 'W' in timer[2]:
                    BlackWhiteArduino.write(timer[2].encode())
                    for index_2,timer_2 in enumerate(timers):
                        if currentTime+2 > timer_2[1]:
                            done_times_indexes.append(index_2)
                    done_times_indexes.append(index)
                elif 'B' in timer[2]:
                    BlackWhiteArduino.write(timer[2].encode())
                    for index_2,timer_2 in enumerate(timers):
                        if currentTime+2>timer_2[1]:
                            done_times_indexes.append(index_2)
                    done_times_indexes.append(index)
                elif 'S' in timer[2]:
                    BlackWhiteArduino.write(timer[2].encode())
                    done_times_indexes.append(index)

    done_times_indexes.sort(reverse=True)
    for done_time_index in done_times_indexes:
        timers.pop(done_time_index)

    cv2.waitKey(1)
