#!/usr/bin/env python3

##################################################
###  -- Node Description 
# This node will be responsible for listening to 
# the visp_auto_tracker node which is looking
# for qr codes in its environment. If it finds 
# a qr code representing a prohibited area it
# will send evasive pathing to the communications 
# topic

##################################################
###  Imports
##################################################

# Standard imports
import signal
import sys
import rospy
from  std_msgs.msg import Int8, String
from geometry_msgs.msg import PoseStamped
from communications.msg import CommsAPI

# Serial communication imports
import serial
import time

# create node as a publisher to the comms api
comms_pub = rospy.Publisher('CommunicationsAPI', CommsAPI, queue_size=10)

##################################################
###  Callbacks
##################################################

# this method will be called when a qr code is localized
def QrCodeCallback(message):
    # retrieve the location of the QR code
    location = rospy.wait_for_message('/visp_auto_tracker/object_position', PoseStamped)

    # if we have a vaild qr code THEN process the label
    if(message.data != ''):
        # stop the robot
        msg = CommsAPI()
        msg.command = "stop"
        comms_pub.publish(msg)

        # IF the qr code is to our left THEN turn right
        if(location.pose.position.x < 0):   
            msg = CommsAPI()
            msg.command = "rotate:90"
            comms_pub.publish(msg)

            msg.command = "forward"
            comms_pub.publish(msg)

        # ELSE the qr code is to our right THEN turn left    
        else:
            msg = CommsAPI()
            msg.command = "rotate:270"
            comms_pub.publish(msg)

            msg.command = "forward"
            comms_pub.publish(msg)

        # wait a for the actions to take place
        time.sleep(5)

    
    

##################################################
###  Methods
##################################################

# main method
def Main():

    # Initialize the node with name "prohibited_areas_node"
    rospy.init_node('prohibited_areas_node', anonymous=True)

    # listen to the visp_auto_tracker code message for
    # qr codes that are localized in the environment
    rospy.Subscriber('/visp_auto_tracker/code_message', String, QrCodeCallback,  queue_size=1)

    # Indefinitely listen to the topics
    rospy.spin()


if __name__ == '__main__':
    Main()