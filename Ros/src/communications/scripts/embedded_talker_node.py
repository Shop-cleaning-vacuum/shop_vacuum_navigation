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
COMMAND_DELIMITER      = '\n'
BRUSH_DATA_ID          = "B"
SEND_BRUSH_DATA_CMD    = b"RB\n"
IR_DATA_ID             = "I"
SEND_IR_DATA_CMD       = "RI"
DISTANCE_DATA_ID       = "D"
SEND_DISTANCE_DATA_CMD = "RD"

##################################################
###  Callbacks
##################################################

##################################################
###  Methods
##################################################

# Main method for this node
def Main():
    # Initialize the node with name "embedded_talker_node"
    rospy.init_node('embedded_talker_node', anonymous=True)

    # Configure the UART serial communication
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

    # Flush the communication line
    ser.flush()

#--------------------------------------------------------------
#--------------------------------------------------------------
#--------------------------------------------------------------

    # indefinitely loop
    while True:
        # Write a "read sensor data" command to the serial port
        ser.write(SEND_BRUSH_DATA_CMD)
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
        time.sleep(1)


#--------------------------------------------------------------
#--------------------------------------------------------------


    # Indefinitely listen to the topics
    rospy.spin()

if __name__ == '__main__':
    Main()