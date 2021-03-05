#!/usr/bin/env python3

##################################################
###  -- Node Description 
# This node will be responsible for 'talking' to 
# the microcontroller controling the systems motors. 
# The node will translate requests to control the
# motors (rotation, translation, stopping, etc.)
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
# the lower-level microcontroller
MSG_DELIMITER   = b'$'
STOP_CMD        = b"S" + MSG_DELIMITER
FWD_CMD         = b"F" + MSG_DELIMITER

# Globally configure the UART serial communication
ser = serial.Serial('/dev/ttyACM3', 9600, timeout=1)

# Flush the communication line
ser.flush()

##################################################
###  Callbacks
##################################################

# This method will be called when there is info on the communcations topic
def CommunicationsCallback(data):
    # If the command is for the motor control micro controller
    # then write the assoicated command to the serial port
    if data.command == "STOP":
        ser.write(STOP_CMD)
    
    elif data.command == "FWD":
        ser.write(FWD_CMD)

##################################################
###  Methods
##################################################

# Main method for this node
def Main():
    # Initialize the node with name "motors_talker_node"
    rospy.init_node('motors_talker_node', anonymous=True)

    # Subscribe to the communications topic inorder to 
    # recieve communication API commands from all of the components
    rospy.Subscriber('CommuncationsAPI', CommsAPI, CommunicationsCallback)

    # Indefinitely listen to the topics
    rospy.spin()

if __name__ == '__main__':
    Main()