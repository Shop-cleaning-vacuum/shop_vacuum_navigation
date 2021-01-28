#!/usr/bin/env python3

##################################################
###  -- Node Description 
# This node is for listening and aggregating debug 
# information from all other components in the project

##################################################
###  Necessary Imports
##################################################
import rospy
from debug.msg import DebugMessage

##################################################
###  Callbacks
##################################################

# This method will be called when there is info on the
# debug info topic
def DebugCallback(data):
    rospy.loginfo(rospy.get_caller_id() + ":\n\t Source: " + data.node_name + " Message: " + data.debug_message + "\n")

##################################################
###  Main Method
##################################################

# Main method for this node
def DebugListener():
    # Initialize the node with name "DebugController"
    rospy.init_node('debug_node', anonymous=True)

    # Subscribe the Debug Info topic inorder to 
    # recieve debug information from all of the components
    rospy.Subscriber('DebugInformation', DebugMessage, DebugCallback)

    # Indefinitely listen to the topic
    rospy.spin()

if __name__ == '__main__':
    DebugListener()