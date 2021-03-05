#!/usr/bin/env python3

##################################################
###  -- Node Description 
# This node will be responsible for 'talking' to 
# the microcontroller collecting sensor data. 
# The node will translate requests for sensor data 
# then transmit them over our serial communication.


##################################################
###  Imports
##################################################

# Standard imports
import rospy
import std_msgs.msg

# Import custom messages
from communications.msg import CommsAPI

# Serial communication imports
import serial
import time

##################################################
###  Defines and Constants
##################################################

# Global constants for the interface protocol between
# the lower-level microcontroller responsible
# for collecting sensor data
MSG_DELIMITER          = b'$'
SEND_BRUSH_DATA_CMD    = b"RB" + MSG_DELIMITER
SEND_IR_DATA_CMD       = b"RI" + MSG_DELIMITER
SEND_DISTANCE_DATA_CMD = b"RD" + MSG_DELIMITER

# Configure the UART serial communication
ser = serial.Serial('/dev/ttyACM2', 9600, timeout=1)

# Flush the communication line
ser.flush()

##################################################
###  Callbacks
##################################################

# This method will be called when there is info on the communcations topic
def CommunicationsCallback(data):
    # Send the given command to the microcontroller
    if data.command == "read:brush":
        ser.write(SEND_BRUSH_DATA_CMD)
    
    elif data.command == "read:ir":
        ser.write(SEND_IR_DATA_CMD)
    
    elif data.command == "read:distance":
        ser.write(SEND_DISTANCE_DATA_CMD)

##################################################
###  Methods
##################################################

# Main method for this node
def Main():
    # Initialize the node with name "sensors_talker_node"
    rospy.init_node('sensors_talker_node', anonymous=True)

    # Subscribe to the communications topic inorder to 
    # recieve communication API commands from all of the components
    rospy.Subscriber('CommuncationsAPI', CommsAPI, CommunicationsCallback)

    # Indefinitely listen to the topics
    rospy.spin()

if __name__ == '__main__':
    Main()