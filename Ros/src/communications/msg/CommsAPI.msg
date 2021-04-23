# This message type serves as a API to the microcontrollers
# responsible for motor control and sensor data collecting
# Using this message type other nodes may request
# the robot to move forward, stop, rotate, send sensor data, etc.

# ------------------ Message Fields ------------------
 
# Name of the api command to execute. The talker nodes
# will interpret this command and send the proper commands
# to the microcontrollers.
string command
