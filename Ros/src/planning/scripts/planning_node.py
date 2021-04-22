#!/usr/bin/env python3

##################################################
###  -- Node Description 
# This node will be used to update the path of the
# autononmous shop vacuum robot

##################################################
###  Necessary Imports
##################################################

# Standard imports
import rospy
import std_msgs.msg
import time

# Imports for custom messages
from debug.msg import DebugMessage, JSON
# from control.msg import MotorAPI, HazardBool

from communications.msg import SensorData, HazardBool


# Imports for sensor data messages
from sensor_msgs.msg import PointCloud2, LaserScan, CompressedImage
import sensor_msgs.point_cloud2 as pc2
##################################################
###  Globals  (
##################################################

current_location = (0,0,0)
isHazard = False
TESTING_FAKE_OUTPUTS = False
##################################################
###  Callbacks
##################################################
def HazardCurrentLocationCallback(data):
    logged_points = ""
    for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
        logged_points += ("\tx : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
        current_location = (p[0],p[1],p[2])
        # global current_location = (p[0],p[1],p[2])
    
    rospy.loginfo_throttle(1,rospy.get_caller_id() + ":\n\t Current Location:" + logged_points)

def HazardBoolCallback(data):
    # print("HazardBoolCallback")
    global isHazard
    global motor_pub
    WasDetected = ""
    if data.detectsHazard == "True":
        if(isHazard == False):
             # Create the motor control command call
            msg = CommsAPI()
            msg.command == "stop"

            # Publish the message
            motor_pub.publish(msg)
            rospy.loginfo_throttle(1,rospy.get_caller_id() + ":\n\t Currently avoiding Hazard")
        
        
        WasDetected = "True"
        isHazard = True
    else:
        if(isHazard == True):
            # Create the motor control command call
            msg = CommsAPI()
            msg.command = "stop"

            # Publish the message
            motor_pub.publish(msg)
        isHazard = False
        WasDetected = "False"
    
    
    rospy.loginfo_throttle(1,rospy.get_caller_id() + ":\n\t HazardDetected:" + WasDetected +" at location (" +str(current_location[0])+", "+str(current_location[1])+", "+str(current_location[2])+")")
    
##################################################
###  Methods
##################################################

def PlanningInit():
    # Intialize the Planning node
    rospy.init_node('planning_node', anonymous=True)
    
    
    # Test all of the ROS message types
    if(True):
        debug_pub,motor_pub,cur_pos_pub,lidar_pub,json_pub,cur_frame_pub,pub_rate,hazard_pub = TestMessagesInit()
     
    # Create a fake planning function for testing of the hazard avoidance that overrides planning

    # PlanningInitialization()

    #Hazard avoidance subscribers are called here
    HazardAvoidanceInitilization()

    Planning(debug_pub,motor_pub,cur_pos_pub,lidar_pub,json_pub,cur_frame_pub,pub_rate,hazard_pub)
    # # Call DebugOut
    # DebugOut("Planning node intialized")

def HazardAvoidance():
    #Hazard function here

    # Create the motor control command call
    msg = CommsAPI()
    msg.command = "rotate:90"

    # Publish the message
    motor_pub.publish(msg)

    # Sleep for 1 second
    # pub_rate.sleep()
    #create a print statement that can be used to test if the planning is being used or if the hazard detection is being used
    rospy.loginfo_throttle(1,rospy.get_caller_id() + ":\n\t Currently avoiding Hazard")

def Planning(debug_pub,motor_pub,cur_pos_pub,lidar_pub,json_pub,cur_frame_pub,pub_rate,hazard_pub):
    starttime = time.time()
    print("time: ",time.time()-starttime)
    #loop that runs all repeating actions in planning
    while not rospy.is_shutdown():
        if not isHazard:
            #Planning function here
            

            # Create the motor control command call
            msg = CommsAPI()
            msg.command = "forward"

            # Publish the message
            motor_pub.publish(msg)

            # Sleep for 1 second
            # pub_rate.sleep()

            #create a print statement that can be used to test if the planning is being used or if the hazard detection is being used
            rospy.loginfo_throttle(1,rospy.get_caller_id() + ":\n\t Currently planning")
        else:
            HazardAvoidance()
        

        

        # change TESTING_FAKE_OUTPUTS to false when you don't need fake data being published
        if(TESTING_FAKE_OUTPUTS):
            if time.time()-starttime > 10:
                print("time: ",time.time()-starttime)
                starttime = time.time()
            if time.time()-starttime < 5:
                msg = HazardBool()
                msg.detectsHazard = "False"
                # print("publishing hazard bool: false")
                hazard_pub.publish(msg)
            else:
                msg = HazardBool()
                msg.detectsHazard = "True"
                # print("publishing hazard bool: true")
                # starttime = time.time()

                hazard_pub.publish(msg)

            TestMessages(debug_pub,motor_pub,cur_pos_pub,lidar_pub,json_pub,cur_frame_pub,pub_rate,hazard_pub)

        



def HazardAvoidanceInitilization():
    # rospy.Subscriber('CurrentPosition', PointCloud2, HazardCurrentLocationCallback)
    rospy.Subscriber('HazardBool', HazardBool, HazardBoolCallback)

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

def TestMessages(debug_pub,motor_pub,cur_pos_pub,lidar_pub,json_pub,cur_frame_pub,pub_rate,hazard_pub):

    # --- Debug message ---

            # Create the debug message
            msg = DebugMessage()
            msg.node_name = "Planning"
            msg.debug_message = "Hello Friend"

            # Publish the debug message 
            debug_pub.publish(msg)

        # --- API motor control message ---

            # # Create the motor control command call
            # msg = MotorAPI()
            # msg.command = "FORWARD"

            # # Publish the message
            # motor_pub.publish(msg)

            # # Sleep for 1 second
            # pub_rate.sleep()

        # --- Point cloud message ---

            # Create the Point cloud object
            point_cloud = [[1.0, 1.0, 0.0],[1.0, 2.0, 0.0]]

            # Format the header
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'

            # Convert the point clouds to a message
            point_cloud_msg = pc2.create_cloud_xyz32(header, point_cloud)

            # Publish the message
            cur_pos_pub.publish(point_cloud_msg)

            # Sleep for 1 second
            pub_rate.sleep()

        # --- Point cloud message ---

            # Create the lidar data message object
            lidar_data_msg = LaserScan()

            # Format the header
            lidar_data_msg.header.stamp = rospy.Time.now()
            lidar_data_msg.header.frame_id = 'laser_frame'

            # Populate the message (fake data)
            num_readings = 10
            laser_frequency = 40

            lidar_data_msg.angle_min = -1.57
            lidar_data_msg.angle_max = 1.57
            lidar_data_msg.angle_increment = 3.14 / num_readings
            lidar_data_msg.time_increment = (1.0 / laser_frequency) / (num_readings)
            lidar_data_msg.range_min = 0.0
            lidar_data_msg.range_max = 100.0
            lidar_data_msg.ranges = []
            lidar_data_msg.intensities = []

            for i in range(0, num_readings):
                lidar_data_msg.ranges.append(1)  
                lidar_data_msg.intensities.append(1)  

            # Publish the message
            lidar_pub.publish(lidar_data_msg)

            # Sleep for 1 second
            pub_rate.sleep()

        # --- JSON message ---

            # Create the JSON message object
            json_msg = JSON()

            # Populate the message
            json_msg.json_string = "{ \"Header\":{ \"Lidar_data\": { \"sensor_1\": \"12345\", }, }, },"

            # Publish the message
            json_pub.publish(json_msg)

            # Sleep for 1 second
            pub_rate.sleep()

        # --- Image messages ---

            # Create the compressed image object
            image_msg = CompressedImage()

            # Store the image
            image_msg.data = [1, 1, 0,1, 2, 0]
            #image_msg.data = image_2msg("/home/patrick/Downloads/env_pic.jpeg")

            # Congigure the message as a jpeg   
            image_msg.format = "jpeg"

            # Publish the message
            cur_frame_pub.publish(image_msg)

            # Sleep for 1 second
            pub_rate.sleep()

# Method to test the sending and recieving of all 
# of our custom ROS messages
def TestMessagesInit():
    global motor_pub
    # 1. ---------- Become a publisher of all the topics ----------

    # Publish to the topic, "DebugInformation"
    debug_pub = rospy.Publisher('DebugInformation', DebugMessage, queue_size=10)

    # Publish to the topic, "MotorControl"
    # motor_pub = rospy.Publisher('MotorControl', MotorAPI, )
    motor_pub = rospy.Publisher('CommunicationsAPI', CommsAPI,queue_size=10)

    # Publish to the topic, "CurrentPosition"
    cur_pos_pub = rospy.Publisher('CurrentPosition', PointCloud2, queue_size=10)

    # Publish to the topic, "LidarData"
    lidar_pub = rospy.Publisher('LidarData', LaserScan, queue_size=10)

    # Publish to the topic, "JSONData"
    json_pub = rospy.Publisher('JSONData', JSON, queue_size=10)

    # Publish to the topic, "CurrentFrame"
    cur_frame_pub = rospy.Publisher('CurrentFrame', CompressedImage, queue_size=10)

    hazard_pub = rospy.Publisher('HazardBool', HazardBool, queue_size=10)

    # Use a publish rate of 1 Hz
    pub_rate = rospy.Rate(10)

    return (debug_pub,motor_pub,cur_pos_pub,lidar_pub,json_pub,cur_frame_pub,pub_rate,hazard_pub)


    # 2. ---------- Send a message to all of the topics ----------
# commented out because I moved this to the main loop above
    # If ROS core is still running -> attempt to send message
    # while not rospy.is_shutdown():

        # # --- Debug message ---

        #     # Create the debug message
        #     msg = DebugMessage()
        #     msg.node_name = "Planning"
        #     msg.debug_message = "Hello Friend"

        #     # Publish the debug message 
        #     debug_pub.publish(msg)

        # # --- API motor control message ---

        #     # Create the motor control command call
        #     msg = MotorAPI()
        #     msg.command = "STOP"

        #     # Publish the message
        #     motor_pub.publish(msg)

        #     # Sleep for 1 second
        #     pub_rate.sleep()

        # # --- Point cloud message ---

        #     # Create the Point cloud object
        #     point_cloud = [[1.0, 1.0, 0.0],[1.0, 2.0, 0.0]]

        #     # Format the header
        #     header = std_msgs.msg.Header()
        #     header.stamp = rospy.Time.now()
        #     header.frame_id = 'map'

        #     # Convert the point clouds to a message
        #     point_cloud_msg = pc2.create_cloud_xyz32(header, point_cloud)

        #     # Publish the message
        #     cur_pos_pub.publish(point_cloud_msg)

        #     # Sleep for 1 second
        #     pub_rate.sleep()

        # # --- Point cloud message ---

        #     # Create the lidar data message object
        #     lidar_data_msg = LaserScan()

        #     # Format the header
        #     lidar_data_msg.header.stamp = rospy.Time.now()
        #     lidar_data_msg.header.frame_id = 'laser_frame'

        #     # Populate the message (fake data)
        #     num_readings = 10
        #     laser_frequency = 40

        #     lidar_data_msg.angle_min = -1.57
        #     lidar_data_msg.angle_max = 1.57
        #     lidar_data_msg.angle_increment = 3.14 / num_readings
        #     lidar_data_msg.time_increment = (1.0 / laser_frequency) / (num_readings)
        #     lidar_data_msg.range_min = 0.0
        #     lidar_data_msg.range_max = 100.0
        #     lidar_data_msg.ranges = []
        #     lidar_data_msg.intensities = []

        #     for i in range(0, num_readings):
        #         lidar_data_msg.ranges.append(1)  
        #         lidar_data_msg.intensities.append(1)  

        #     # Publish the message
        #     lidar_pub.publish(lidar_data_msg)

        #     # Sleep for 1 second
        #     pub_rate.sleep()

        # # --- JSON message ---

        #     # Create the JSON message object
        #     json_msg = JSON()

        #     # Populate the message
        #     json_msg.json_string = "{ \"Header\":{ \"Lidar_data\": { \"sensor_1\": \"12345\", }, }, },"

        #     # Publish the message
        #     json_pub.publish(json_msg)

        #     # Sleep for 1 second
        #     pub_rate.sleep()

        # # --- Image messages ---

        #     # Create the compressed image object
        #     image_msg = CompressedImage()

        #     # Store the image
        #     image_msg.data = [1, 1, 0,1, 2, 0]
        #     #image_msg.data = image_2msg("/home/patrick/Downloads/env_pic.jpeg")

        #     # Congigure the message as a jpeg   
        #     image_msg.format = "jpeg"

        #     # Publish the message
        #     cur_frame_pub.publish(image_msg)

        #     # Sleep for 1 second
        #     pub_rate.sleep()

if __name__ == '__main__':
    try:
        print("startubg planning ")
        PlanningInit()
        print("initialized ")
    except rospy.ROSInterruptException:
        pass
