#! /bin/bash

# -------------------------------------
# ------------- Globals ---------------
# -------------------------------------

TEST_FILE=../testing/sensor_output.txt
ROS=../Ros/

# -------------------------------------
# ----------- Main Method -------------
# -------------------------------------

# change to the ROS directory
cd $ROS

# source the terminal
source devel/setup.bash

# start the ros nodes
roslaunch communications sensors.launch &

# wait 5 seconds for ROS to start up
sleep 10s

# print the recieved sensor data to a test file
rostopic echo /BumpSensorData >> $TEST_FILE &
rostopic echo /IRSensorData >> $TEST_FILE &
rostopic echo /DistanceSensorData >> $TEST_FILE &
rostopic echo /BrushCurrentData >> $TEST_FILE &
rostopic echo /BrushTempData >> $TEST_FILE &
rostopic echo /BrushPositionSensorData >> $TEST_FILE &

# wait 5 seconds for previous task to finish
sleep 5s

# tell the system to read the sensor data
rostopic pub /CommunicationsAPI communications/CommsAPI "{command: 'read:sensors', param0: 0, param1: 0, param2: 0, param3: 0, param4: 0}"

