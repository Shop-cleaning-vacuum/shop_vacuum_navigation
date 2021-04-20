#!/usr/bin/env python3

##################################################
###  -- Node Description 
# This node will be responsible for listening to 
# communication coming from the microcontroller
# responsible for motor control. The node
# will then publish this recieved data to ROS

##################################################
###  Imports
##################################################

# Standard imports
import rospy
import std_msgs.msg

# Serial communication imports
import serial
import time

##################################################
###  Defines and Constants
##################################################

# Global constants for the interface protocol between
# the lower-level microcontroller 
MSG_DELIMITER  = '$'
ACK            = "A"

# Globally configure the UART serial communication
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# Flush the communication line
ser.flush()

##################################################
###  Callbacks
##################################################

##################################################
###  Methods
##################################################

# Main method for this node
def Main():
    # Initialize the node with name "motors_listener_node"
    rospy.init_node('motors_listener_node', anonymous=True)

    # indefinitely listen to the serial port
    while True:
        # Read in the current response 
        line = ser.readline().decode('utf-8').rstrip()

        # If the response was an ack, then print it out
        if line == ACK:
            print(line)

    
    
if __name__ == '__main__':
    Main()