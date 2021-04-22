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
from communications.msg import SensorData, HazardBool

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
TABLE_START       = "S"
TABLE_END         = "E"
NUM_BUMP_SENSORS            = 3
NUM_IR_SENSORS              = 5
NUM_DISTANCE_SENSORS        = 4
NUM_BRUSH_CURRENT_SENSORS   = 3
NUM_BRUSH_TEMP_SENSORS      = 3
NUM_BRUSH_POSITION_SENSORS  = 3


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

#  --------------------------------------
#  ----  Main method for this node   ----
def Main():
    # Initialize the node with name "sensors_listener_node"
    rospy.init_node('sensors_listener_node', anonymous=True)

    # Create sensor topics to publish too
    bump_pub        = rospy.Publisher('BumpSensorData', SensorData, queue_size=10)
    ir_pub          = rospy.Publisher('IRSensorData', SensorData, queue_size=10)
    distance_pub    = rospy.Publisher('DistanceSensorData', SensorData, queue_size=10)
    brush_c_pub     = rospy.Publisher('BrushCurrentData', SensorData, queue_size=10)
    brush_t_pub     = rospy.Publisher('BrushTempData', SensorData, queue_size=10)
    brush_p_pub     = rospy.Publisher('BrushPositionSensorData', SensorData, queue_size=10)
    hazard_pub      = rospy.Publisher('HazardBool', HazardBool, queue_size=10)

    # Indefinitely listen to the serial port
    while True:
        # Read in the current response 
        line = ser.readline().decode('utf-8').rstrip()
        
        # If we recieve the 'start_table' character, then start
        # reading the table from the microcontroller
        if line == TABLE_START:
            ReadTable(bump_pub, ir_pub, distance_pub, brush_c_pub, brush_t_pub,brush_p_pub,hazard_pub)



#  -----------------------------------------------
#  --- Method to read the table of sensor data ---
#  ---   from the low-level microcontroller    ---
def ReadTable(bump_pub, ir_pub, distance_pub, brush_c_pub, brush_t_pub,brush_p_pub,hazard_pub):
    # Read all the sensor data
    bump_sensor_message             = ReadSensorData(NUM_BUMP_SENSORS)
    ir_sensor_message               = ReadSensorData(NUM_IR_SENSORS)
    distance_sensor_message         = ReadSensorData(NUM_DISTANCE_SENSORS)
    brush_current_sensor_message    = ReadSensorData(NUM_BRUSH_CURRENT_SENSORS)
    brush_temp_sensor_message       = ReadSensorData(NUM_BRUSH_TEMP_SENSORS)
    brush_position_sensor_message   = ReadSensorData(NUM_BRUSH_POSITION_SENSORS)

    # Publish all the sensor data
    bump_pub.publish(bump_sensor_message)
    ir_pub.publish(ir_sensor_message)
    distance_pub.publish(distance_sensor_message)
    brush_c_pub.publish(brush_current_sensor_message)
    brush_t_pub.publish(brush_temp_sensor_message)
    brush_p_pub.publish(brush_position_sensor_message)

    # Publish current HazardBool based on sensor data
    msg = HazardBool()
    msg.detectsHazard = "False"
    for bump_sensor in bump_sensor_message.sensors:
        if bump_sensor:
            msg.detectsHazard = "True"
    if brush_temp_sensor_message[0] == 100:
        msg.detectsHazard = "True"
    hazard_pub.publish(msg)


        

#  --------------------------------------
#  ----     Read in sensor data      ----
def ReadSensorData(num_sensors):
    # Read in each byte from the serial line
    container = []
    for x in range(num_sensors):
        container.append(ReI adByte())

    # create and populate the sensor data message
    sensor_message             = SensorData()
    sensor_message.num_sensors = num_sensors 
    sensor_message.sensors     = container

    # return the built up message
    return sensor_message


#  --------------------------------------
#  ---   Read line from serial line   ---
def ReadByte():
    return int(ser.readline().decode('utf-8').rstrip())
    
if __name__ == '__main__':
    Main()