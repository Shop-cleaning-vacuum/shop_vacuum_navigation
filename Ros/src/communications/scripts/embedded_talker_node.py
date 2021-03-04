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

    # Indefinitely listen to the topics
    rospy.spin()

if __name__ == '__main__':
    Main()