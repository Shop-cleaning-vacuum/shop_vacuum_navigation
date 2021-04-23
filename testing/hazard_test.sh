#! /bin/bash

# -------------------------------------
# ------------- Globals ---------------
# -------------------------------------

TEST_FILE=../testing/output/hazard_output.txt
ROS=../Ros/

# -------------------------------------
# ----------- Main Method -------------
# -------------------------------------

# remove the old test file
rm -f $TEST_FILE

# change to the ROS directory
cd $ROS

# source the terminal
source devel/setup.bash

# start the ros nodes
roslaunch communications motors.launch &
roslaunch communications sensors.launch &

# wait 10 seconds for ROS to start up
sleep 10s

# print the recieved motor data to a test file
rostopic echo /LeftMotor >> $TEST_FILE &
rostopic echo /RightMotor >> $TEST_FILE &

# wait 1 second for previous task to finish
sleep 1s

# tell the system to read the motor data
printf "Left & Right Motor Status:\n" >> $TEST_FILE
rostopic pub /CommunicationsAPI communications/CommsAPI "{command: 'read:motors'}" 

# wait 1 second for previous task to finish
sleep 1s

# tell the system to read the motor data
printf "Left & Right Motor Status:\n" >> $TEST_FILE
rostopic pub /CommunicationsAPI communications/CommsAPI "{command: 'read:motors'}" & 
