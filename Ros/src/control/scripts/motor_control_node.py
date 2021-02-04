#!/usr/bin/env python3

##################################################
###  -- Node Description 
# This node is for listening for calls to the 
# Motor control topic and then routing the command
# to the low level processor to control the motors
# of the shop cleaning vacuum

##################################################
###  Necessary Imports
##################################################
import rospy
from control.msg import MotorAPI

##################################################
###  Callbacks
##################################################

# This method will be called when there is info on the
# motor control topic
def MorotControlCallback(data):
    rospy.loginfo(rospy.get_caller_id() + ":\n\tCommand: " + data.command + "\n")

##################################################
###  Main Method
##################################################

# Main method for this node
def MotorControlListener():
    # Initialize the node with name "motor_control_node"
    rospy.init_node('motor_control_node', anonymous=True)

    # Subscribe to the motor control topic inorder to 
    # recieve motor control commands from all of the components
    rospy.Subscriber('MotorControl', MotorAPI, MorotControlCallback)

    # Indefinitely listen to the topic
    rospy.spin()

if __name__ == '__main__':
    MotorControlListener()