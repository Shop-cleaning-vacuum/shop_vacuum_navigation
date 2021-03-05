#!/usr/bin/env python3

##################################################
###  -- Node Description 
# This node will be responsible for 'talking' to 
# the microcontroller for collecting sensor data. 
# The node will translate requests for sensor data 
# then transmit them over our serial communication.

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
###  Callbacks
##################################################

##################################################
###  Methods
##################################################

# Main method for this node
def Main():
    # Initialize the node with name "embedded_listener_node"
    rospy.init_node('embedded_listener_node', anonymous=True)

    # Configure the UART serial communication
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

    # Flush the communication line
    ser.flush()

    # Indefinitely listen to the topics
    rospy.spin()

if __name__ == '__main__':
    Main()