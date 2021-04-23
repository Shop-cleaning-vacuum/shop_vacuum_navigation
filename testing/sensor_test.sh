#! /bin/bash

# -------------------------------------
# ------------- Globals ---------------
# -------------------------------------

TEST_FILE=../testing/ouput/sensor_output.txt
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
roslaunch communications sensors.launch &

# wait 10 seconds for ROS to start up
sleep 10s

# print the recieved sensor data to a test file
rostopic echo /BumpSensorData >> $TEST_FILE &
rostopic echo /IRSensorData >> $TEST_FILE &
rostopic echo /DistanceSensorData >> $TEST_FILE &
rostopic echo /BrushCurrentData >> $TEST_FILE &
rostopic echo /BrushTempData >> $TEST_FILE &
rostopic echo /BrushPositionSensorData >> $TEST_FILE &

# wait 1 seconds for previous task to finish
sleep 1

# tell the system to read the sensor data
rostopic pub /CommunicationsAPI communications/CommsAPI "{command: 'read:sensors', param0: 0, param1: 0, param2: 0, param3: 0, param4: 0}"

