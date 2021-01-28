#!/usr/bin/env python3

##################################################
###  -- Node Description 
# This node will be used to update the path of the
# autononmous shop vacuum robot

##################################################
###  Necessary Imports
##################################################
import rospy
from std_msgs.msg import String

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
    pub = rospy.Publisher('DebugInformation', String, queue_size=10)

    # Use a publish rate of 10hz
    pub_rate = rospy.Rate(10)

    # If ROS core is still running -> attempt to send message
    while not rospy.is_shutdown():
            hello_str = "hello world"
            pub.publish(hello_str)
            pub_rate.sleep()



if __name__ == '__main__':
    try:
        PlanningInit()
    except rospy.ROSInterruptException:
        pass
