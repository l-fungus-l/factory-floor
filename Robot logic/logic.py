from time import perf_counter, sleep
import cv2

# Getting the serials done: AA
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

timeUntilPush = 0.5
timeUntilWhiteSorted = 0.5
timeUntilBlackSorted = 0.5
timeout = 5

# LOCAL VARIABLES

# the time at which circle was pushed on the belt, none if there is no circle on the belt
whenPuckOnBelt = None

# Create phototransistors data
catapultPhotoData = 0
armPhotoData1 = 0
armPhotoData2 = 0

# What is the states of the servos, false is default, true is pushed
whiteServoExtended = False
blackServoExtended = False



# The color of the circle currently on the belt
circleColor = "NONE"

whenPuckOnBelt = perf_counter()

while True:
    # Get the phototransistor data
    a = BlackWhiteArduino.readline().decode().strip()
    if a.split(': ')[0] == 'S3':
        catapultPhotoData = int(a.split(': ')[1])

    # Try and get the color of the thing on the belt
    a = detecter.detect(esp_cam_1)
    if not a == None:
        circleColor = a
    
    # If there is a circle on the belt and it is white, and the white servo is not extended, extend it
    if circleColor == "WHITE" and not whiteServoExtended:
        # Extend the white servo
        whiteServoExtended = True
        # Push the circle
        BlackWhiteArduino.write(bytes(f'W {whiteAnglePush}'))
    
    if circleColor == "BLACK" and not blackServoExtended:
        # Extend the black servo
        blackServoExtended = True
        # Push the circle
        BlackWhiteArduino.write(bytes(f'B {blackAnglePush}'))

    # If one of the times is up, retract the servo
    if perf_counter() - whenPuckOnBelt > timeUntilPush:
        