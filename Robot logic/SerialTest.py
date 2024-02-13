import serial
import logging
from time import sleep
import time
import cv2


whiteAngleDefault = 90
whiteAnglePush = 145

blackAngleDefault = 0
blackAnglePush = 45
# Getting the serials done:
BlackWhiteArduino = serial.Serial(
    'COM5', 9600, timeout=0.1)  # STUFF THAT SHOULD BE CHANGED
#sleep in order for the arduinos to do thei setups
ArmPushArduino = serial.Serial('COM6', 9600)  # SAME HERE
sleep(2)
ArmPushArduino.write("P 20\n".encode())
sleep(2)
ArmPushArduino.write("P 0\n".encode())
#FactoryPushArduino = serial.Serial('/dev/ttyACMO', 9600)  # SAME HERE

logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(levelname)s %(name)s %(message)s')
cap = cv2.VideoCapture(1)

while True:
    # Empty the buffer to prevent buildup of text
    start = time.time()
    while time.time() - start < 0.1:
        ArmPushArduino.read()

    # ArmPushArduino.
    try:
        sleep(0.3)
        a = ArmPushArduino.readline()
        factoryData = a.decode().strip()
        parts = factoryData.split()

    except UnicodeDecodeError:
        factoryData = None
    print (factoryData)
