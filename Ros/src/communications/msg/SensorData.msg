# This message servers as a container for sensor
# data reported by the low level microcontroller

# ------------------ Message Fields ------------------

# name of the type of sensors
string sensor_name

# number of sensors in this message
int8 num_sensors

# array of the sensor data
int8[] sensors