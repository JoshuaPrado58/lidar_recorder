#!usr/bin/env python3

# The above line is called a shebang. It tells the computer 
#what interpreter to use to run the file.

import rospy
#By importing rospy, we accesss all the entire ROS Library

from sensor_msgs.msg import LaserScan
#Sensor_msgs is a package that contains messages for commonly used sensors
#LaserScan is a message type that contains data from a LIDAR sensor

from std_msgs.msg import String
#std_msgs is a package that contains messages for common data types

import csv
#csv is a library that allows us to read and write csv files
#Wil

import signal
#Signal will allow us to use control-c to stop the program

#We will create a class called Lidar_Recorder
class lidar_recorder:
    #Class is a blueprint for an object, in this case, Lidar_Recorder
    
    def __init__(self):
    #The __init__ function is called when an object of the class is created
    #Self refers to the current object
        
        rospy.init_node('lidar_recorder_subscriber')
        #rospy.init_node is a function that initializes the node
        #The node is called lidar_recorder_subscriber

        # Now we will Subscribe to the /scan and /scan_filtered topics
        
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        #self.scan_subscriber is a variable that stores the subscriber
        #rospy.Subscriber is a function that subscribes to a topic
        #The topic is called /scan
        #The message type is LaserScan
        #The callback function is scan_callback
        
        self.scan_filtered_subscriber = rospy.Subscriber('/scan_filtered', LaserScan, self.scan_filtered_callback)
        #Same as above, but for /scan_filtered

    def scan_callback(self, scan_message):
    #This is the callback function for /scan
    #Self refers to the current object
    #scan_message is the message that is received from the topic
    #This function will be called every time a message is received from /scan

        # Example: Print the range data from the laser scan
        #Change this Exammple to better suit what is required for the project
        ranges = scan_message.ranges
        print("Received laser scan data from /scan:")
        print(ranges)
    
    def scan_filtered_callback(self, scan_filtered_message):
    # This is the same as scan_callback, but for /scan_filtered

        # Example: Print the range data from the filtered laser scan
        ranges = scan_filtered_message.ranges
        print("Received laser scan data from /scan_filtered:")
        print(ranges)

    def run(self):
    #This function will run the node until control-c is pressed
        rospy.spin()

if __name__ == '__main__':
    subscriber = lidar_recorder()
    subscriber.run()