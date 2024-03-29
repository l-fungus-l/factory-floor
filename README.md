# Embedded Systems Experience:
This course comprised of a SCRUM-based team project, where the aim was to make an automaton that would sort circular disks based on their color. To achieve this our team designed a system of a RaspberryPi, connected to multiple Arduino microcontrollers that in turn control each major part of the machine. The aim of this was to improve the concurrency of the system as well as segmenting it for manageability reasons. Additionally to giving me a base understanding of electronic circuits, soldering, sensors, and motors, I contributed most to two imperative aspects of the project:
 - The communication between the Arduino boards and the RaspberryPi through Serial code, as well as communication between our system and others through the use of the MQTT messaging protocol
 
 [Serial and MQTT communication example file](./ArduinoRaspberryCommunication/PuckPushFactoryFloor/src/main.cpp)
 
 - Image processing in the form of circle recognition, as well as recognition of its relative color through the Canny library and other highly specific methods

[Detailed explanation of image processing](./Robot%20logic/Programming%20Logbook.ipynb)

In the folders are avaialable a multitude of folders and files that I contributed to, as well as documentation explaining in further detail the process and methodology used for the group done by all members.
