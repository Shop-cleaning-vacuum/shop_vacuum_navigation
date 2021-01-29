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
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

##################################################
###  Callbacks
##################################################

# This method will be called when there is info on the
# debug info topic
def DebugCallback(data):
    rospy.loginfo(rospy.get_caller_id() + ":\n\t Source: " + data.node_name + " Message: " + data.debug_message + "\n")

# This method will be called when there is info on the current position topic
def CurrentLocationCallback(data):
    rospy.loginfo(rospy.get_caller_id() + ":\n\t Current Location:")
    for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
        print ("\tx : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
    print("\n")

##################################################
###  Main Method
##################################################

# Main method for this node
def DebugListener():
    # Initialize the node with name "DebugController"
    rospy.init_node('debug_node', anonymous=True)

    # Subscribe to the Debug Info topic inorder to 
    # recieve debug information from all of the components
    rospy.Subscriber('DebugInformation', DebugMessage, DebugCallback)

    # Subscribe to the current position topic inorder to 
    # recieve point cloud information on the current location
    rospy.Subscriber('CurrentPosition', PointCloud2, CurrentLocationCallback)

    # Indefinitely listen to the topic
    rospy.spin()

if __name__ == '__main__':
    DebugListener()