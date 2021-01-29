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
from control.msg import MotorAPI

##################################################
###  Callbacks
##################################################

##################################################
###  Methods
##################################################

def PlanningInit():
    # Intialize the Planning node
    rospy.init_node('planning_node', anonymous=True)
    
    # Test all of the ROS message types
    TestMessages()

    # # Call DebugOut
    # DebugOut("Planning node intialized")

# Method to publish to the debug information 
# topic and send the passed in message
def DebugOut(message):
    # Publish to the topic, "DebugInformation"
    pub = rospy.Publisher('DebugInformation', DebugMessage, queue_size=10)

    # If ROS core is still running -> attempt to send message
    while not rospy.is_shutdown():
            # Create the debug message
            msg = DebugMessage()
            msg.node_name = "Planning"
            msg.debug_message = message

            # Publish the debug message 
            pub.publish(msg)

# Method to test the sending and recieving of all 
# of our custom ROS messages
def TestMessages():

    # 1. ---------- Become a publisher of all the topics ----------

    # Publish to the topic, "DebugInformation"
    debug_pub = rospy.Publisher('DebugInformation', DebugMessage, queue_size=10)

    # Publish to the topic, "MotorControl"
    motor_pub = rospy.Publisher('MotorControl', MotorAPI, queue_size=10)

    # Use a publish rate of 1 Hz
    pub_rate = rospy.Rate(1)


    # 2. ---------- Send a message to all of the topics ----------

    # If ROS core is still running -> attempt to send message
    while not rospy.is_shutdown():

        # --- Debug message ---

        # Create the debug message
        msg = DebugMessage()
        msg.node_name = "Planning"
        msg.debug_message = "Hello Friend"

        # Publish the debug message 
        debug_pub.publish(msg)

        # Sleep for 1 second
        pub_rate.sleep()

        # --- API motor control message ---

        # Create the motor control command call
        msg = MotorAPI()
        msg.command = "STOP"

        # Publish the message
        motor_pub.publish(msg)

        # Sleep for 1 second



if __name__ == '__main__':
    try:
        PlanningInit()
    except rospy.ROSInterruptException:
        pass
