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
from  std_msgs.msg import Int8

# Serial communication imports
import serial
import time

##################################################
###  Defines and Constants
##################################################

# Global constants for the interface protocol between
# the lower-level microcontroller 
MSG_DELIMITER     = '$'
TABLE_START       = "S"
TABLE_END         = "E"

# Globally configure the UART serial communication
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

# Flush the communication line
ser.flush()

##################################################
###  Callbacks
##################################################

##################################################
###  Methods
##################################################

#  -----------------------------------------------
#  --- Method to read the table of motor data ---
#  ---   from the low-level microcontroller    ---
def Main():
    # Initialize the node with name "motors_listener_node"
    rospy.init_node('motors_listener_node', anonymous=True)

    # Create sensor topics to publish too
    right_motor_pub = rospy.Publisher('RightMotor', Int8, queue_size=10)
    left_motor_pub  = rospy.Publisher('LeftMotor', Int8, queue_size=10)

    # indefinitely listen to the serial port
    while True:
        # Read in the current response 
        line = ser.readline().decode('utf-8').rstrip()

        # If we recieve the 'start_table' character, then start
        # reading the table from the microcontroller
        if line == TABLE_START:
            ReadTable(right_motor_pub, left_motor_pub)


#  -----------------------------------------------
#  --- Method to read the table of motor data ---
#  ---   from the low-level microcontroller    ---
def ReadTable(right_motor_pub, left_motor_pub):
    # Read all the data
    right_motor_message = ReadByte()
    left_motor_message  = ReadByte()

    # publish all the data
    right_motor_pub.publish(right_motor_message)
    left_motor_pub.publish(left_motor_message)
    
    
#  --------------------------------------
#  ---   Read line from serial line   ---
def ReadByte():
    b            = int(ser.readline().decode('utf-8').rstrip())
    message      = Int8()
    message.data = b

    return message 


if __name__ == '__main__':
    Main()