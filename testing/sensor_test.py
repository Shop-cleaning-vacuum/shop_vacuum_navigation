# -------------------------------------
# ------------- Imports ---------------
# -------------------------------------

import csv # for writing to a csv file
import os  # for using the command line 

# -------------------------------------
# ----------- Main Method -------------
# -------------------------------------

# change to the ROS directory
os.chdir("../Ros/")

# make the libarary
os.system("catkin_make")

# start the sensor nodes
os.system("launch ")
