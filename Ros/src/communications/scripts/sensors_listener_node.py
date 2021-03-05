#!/usr/bin/env python3

##################################################
###  -- Node Description 
# This node will be responsible for listening to 
# communication coming from the microcontroller
# responsible for collecting sensor data. The node
# will then publish this sensor data to ros

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
# the lower-level microcontroller responsible
# for collecting sensor data
MSG_DELIMITER     = '$'
BRUSH_DATA_ID     = "B"
IR_DATA_ID        = "I"
DISTANCE_DATA_ID  = "D"

# Globally configure the UART serial communication
ser = serial.Serial('/dev/ttyACM2', 9600, timeout=1)

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
    # Initialize the node with name "sensors_listener_node"
    rospy.init_node('sensors_listener_node', anonymous=True)

    # Indefinitely listen to the serial port
    while True:
        # Read in the current response 
        line = ser.readline().decode('utf-8').rstrip()

        # If it is valid data then publish it to ROS
        if line == BRUSH_DATA_ID:
            ReadBrush()

        elif line == IR_DATA_ID:
            ReadIR()

        elif line == DISTANCE_DATA_ID:
            ReadDistance()

# Method to read IR sensor data from the serial port
def ReadIR():
    # Read data until you reach the MSG_DELIMITER 
    line = ser.readline().decode('utf-8').rstrip()
    
    while line != MSG_DELIMITER:
        print(line)
        line = ser.readline().decode('utf-8').rstrip()

# Method to read brush sensor data from the serial port
def ReadBrush():
    # Read data until you reach the MSG_DELIMITER
    line = ser.readline().decode('utf-8').rstrip()

    while line != MSG_DELIMITER:
        print(line)
        line = ser.readline().decode('utf-8').rstrip()

# Method to read distance sensor data from the serial port
def ReadDistance():
    # Read data until you reach the MSG_DELIMITER
    line = ser.readline().decode('utf-8').rstrip()

    while line != MSG_DELIMITER:
        print(line)
        line = ser.readline().decode('utf-8').rstrip()
    
    
if __name__ == '__main__':
    Main()