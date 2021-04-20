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
READ_TABLE_CMD  = b"R" + MSG_DELIMITER
WRITE_TABLE_CMD = b"W" + MSG_DELIMITER
SPEED_STOPPED   = 0
SPEED_SLOW      = 1
SPEED_MEDIUM    = 5
SPEED_FULL      = 10

# Globally configure the UART serial communication
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

# Flush the communication line
ser.flush()

##################################################
###  Callbacks
##################################################

# This method will be called when there is info on the communcations topic
def CommunicationsCallback(data):
    # Send the given command to the microcontroller
    if data.command == "read:motors":
        ser.write(READ_TABLE_CMD)
    
    elif data.command == "forward":
        UpdateTable(SPEED_MEDIUM.to_bytes(1, 'big'), SPEED_MEDIUM.to_bytes(1, 'big')) 

    elif data.command == "stop":
        UpdateTable(SPEED_STOPPED.to_bytes(1, 'big'), SPEED_STOPPED.to_bytes(1, 'big'))
        
    elif data.command == "rotate:90":
        # turn on the left motor
        UpdateTable(SPEED_MEDIUM.to_bytes(1, 'big'), SPEED_STOPPED.to_bytes(1, 'big')) 

        # wait a predefined amout of time
        time.sleep(1)

        # stop the robot once it has rotated
        UpdateTable(SPEED_STOPPED.to_bytes(1, 'big'), SPEED_STOPPED.to_bytes(1, 'big'))

##################################################
###  Methods
##################################################

# Main method for this node
def Main():
    # Initialize the node with name "motors_talker_node"
    rospy.init_node('motors_talker_node', anonymous=True)

    # Subscribe to the communications topic inorder to 
    # recieve communication API commands from all of the components
    rospy.Subscriber('CommunicationsAPI', CommsAPI, CommunicationsCallback)

    # Indefinitely listen to the topics
    rospy.spin()


# Method to update the motor control table
def UpdateTable(param1, param2):
    ser.write(WRITE_TABLE_CMD)
    ser.write(param1)
    ser.write(param2)

if __name__ == '__main__':
    Main()