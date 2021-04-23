#! /bin/bash

# -------------------------------------
# ------------- Globals ---------------
# -------------------------------------

TEST_FILE=../testing/output/motor_output.txt
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

# wait 10 seconds for ROS to start up
sleep 10s

# print the recieved motor data to a test file
rostopic echo /LeftMotor >> $TEST_FILE &
rostopic echo /RightMotor >> $TEST_FILE &

# wait 1 second for previous task to finish
sleep 1s

# tell the system to send navigation data to the motor control board
rostopic pub /CommunicationsAPI communications/CommsAPI "{command: 'stop', param0: 0, param1: 0, param2: 0, param3: 0, param4: 0}" &

# wait 3 seconds for previous task to finish
sleep 3s

# tell the system to read the motor data
printf "Left & Right Motor Status:\n" >> $TEST_FILE
rostopic pub /CommunicationsAPI communications/CommsAPI "{command: 'read:motors', param0: 0, param1: 0, param2: 0, param3: 0, param4: 0}" &

# wait 3 seconds for previous task to finish
sleep 3s

# tell the system to send navigation data to the motor control board
rostopic pub /CommunicationsAPI communications/CommsAPI "{command: 'forward', param0: 0, param1: 0, param2: 0, param3: 0, param4: 0}" &

# wait 3 seconds for previous task to finish
sleep 3s

# tell the system to send navigation data to the motor control board
printf "\nLeft & Right Motor Status:\n" >> $TEST_FILE
rostopic pub /CommunicationsAPI communications/CommsAPI "{command: 'read:motors', param0: 0, param1: 0, param2: 0, param3: 0, param4: 0}" 
