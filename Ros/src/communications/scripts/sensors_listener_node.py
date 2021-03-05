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

# Imports for sensor data messages
from sensor_msgs.msg import PointCloud2, LaserScan

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
ser = serial.Serial('/dev/ttyACM4', 9600, timeout=1)

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

    # Publish to the topic, "IRData"
    ir_pub = rospy.Publisher('IRData', LaserScan, queue_size=10)

    # Indefinitely listen to the serial port
    while True:
        # Read in the current response 
        line = ser.readline().decode('utf-8').rstrip()

        # If it is valid data then publish it to ROS
        if line == IR_DATA_ID:
            ir_pub.publish(ReadIR())

        elif line == BRUSH_DATA_ID:
            ReadBrush()

        elif line == DISTANCE_DATA_ID:
            ReadDistance()

# Method to read IR sensor data from the serial port
def ReadIR():
    # Create the laser data message object
    laser_scan_msg = LaserScan()

    # Format the header
    laser_scan_msg.header.stamp = rospy.Time.now()
    laser_scan_msg.header.frame_id = 'laser_frame'

    # Read in the first byte from the port and 
    # create an array to hold the data
    data = []
    line = ser.readline().decode('utf-8').rstrip()
    data.append(line)

    # Read data until you reach the MSG_DELIMITER 
    while line != MSG_DELIMITER:
        # read in the current byte and append it to the array
        line = ser.readline().decode('utf-8').rstrip()
        data.append(line)

    # Convert the data to the laser_scan_msg
    laser_scan_msg.angle_min = float(data[0])
    laser_scan_msg.angle_max = float(data[1])
    laser_scan_msg.range_min = float(data[2])
    laser_scan_msg.range_max = float(data[3])

    # Return the built up laser scan message
    return laser_scan_msg
    

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