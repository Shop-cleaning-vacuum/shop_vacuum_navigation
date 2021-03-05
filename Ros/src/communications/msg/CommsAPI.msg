# This message type serves as a API to the microcontrollers
# responsible for motor control and sesnor data collecting
# Using this message type other nodes may request
# the robot to move forward, stop, rotate, send sensor data, etc.

# ------------------ Message Fields ------------------
 
# Name of the api command to execute. The microcontrollers
# will switch over this value and execute the command
# if a valid value was passed in
string command

# Optional paramters to be used for the api call
int8 param0
int8 param1
int8 param2
int8 param3
int8 param4