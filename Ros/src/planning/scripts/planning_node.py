#!/usr/bin/env python3

##################################################
###  -- Node Description 
# This node will be used to update the path of the
# autononmous shop vacuum robot

##################################################
###  Necessary Imports
##################################################
import rospy
from debug.msg import DebugMessage

##################################################
###  Callbacks
##################################################

##################################################
###  Methods
##################################################

def PlanningInit():
    # Intialize the Planning node
    rospy.init_node('planning_node', anonymous=True)
    
    # Call DebugOut
    DebugOut()

# Method to publish to the debug information 
# topicand send debug info periodically
def DebugOut():
    # Publish to the topic, "DebugInformation"
    pub = rospy.Publisher('DebugInformation', DebugMessage, queue_size=10)

    # Use a publish rate of 10hz
    pub_rate = rospy.Rate(10)

    # If ROS core is still running -> attempt to send message
    while not rospy.is_shutdown():
            # Create the debug message
            msg = DebugMessage()
            msg.node_name = "Planning"
            msg.debug_message = "How's it going friend"

            # Publish the debug message then sleep for 10 hz
            pub.publish(msg)
            pub_rate.sleep()

if __name__ == '__main__':
    try:
        PlanningInit()
    except rospy.ROSInterruptException:
        pass
