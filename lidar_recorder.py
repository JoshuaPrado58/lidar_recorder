#!usr/bin/env python3

#The above line is called a shebang. It tells the computer 
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
        
        rospy.init_node('lidar_recorder')
        #rospy.init_node is a function that initializes the node
        #The node is called lidar_recorder

        #Now we will Subscribe to the /scan and /scan_filtered topics
        
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        #self.scan_subscriber is a variable that stores the subscriber
        #rospy.Subscriber is a function that subscribes to a topic
        #The topic is called /scan
        #The message type is LaserScan
        #The callback function is scan_callback
        #The callback function is called every time a message is received from the topic
        #Its purpose is for us to be able to see the data from the topic
        
        self.scan_filtered_subscriber = rospy.Subscriber('/scan_filtered', LaserScan, self.scan_filtered_callback)
        #Same as above, but for /scan_filtered


        self.object_detected_publisher = rospy.Publisher('/object_detected', String, queue_size=10)
        #/object_detected is the topic that we will publish to
        #String is the message type
        #queue_size is the max number of messages that can be stored in que

        self.csv_file = open('lidar_data.csv', 'w')
        #Open a csv file called lidar_data.csv
        #w means that we will write to the file

        self.csv_writer = csv.writer(self.csv_file)
        #csv.writer is a function that will allow us to write to the csv file


    def scan_callback(self, scan_message):
    #This is the callback function for /scan
    #Self refers to the current object
    #scan_message is the message that is received from the topic
    #This function will be called every time a message is received from /scan

        # Example: Print the range data from the laser scan
        #Change this Example to better suit what is required for the project
        ranges = scan_message.ranges
        print("Received laser scan data from /scan:")
        print(ranges)

        # Example: Write the range data to a csv file
        self.csv_writer.writerow(['scan'] + ranges)
        #Write the data to the csv file
        #The first column will be called scan
        #The rest of the columns will be the range data

    
    def scan_filtered_callback(self, scan_filtered_message):
    # This is the same as scan_callback, but for /scan_filtered
    #Scan_filtered gives us the data from the LIDAR after it has been filtered
    #By filtered, we mean that the data has been processed to remove noise

        # Example: Print the range data from the filtered laser scan
        ranges = scan_filtered_message.ranges
        print("Received laser scan data from /scan_filtered:")
        print(ranges)

        self.csv_writer.writerow(['scan_filtered'] + ranges)
        #Same as above, but for /scan_filtered

    def run(self):
    #This function will run the node until control-c is pressed
        rospy.spin()

if __name__ == '__main__':
    subscriber = lidar_recorder()
    subscriber.run()