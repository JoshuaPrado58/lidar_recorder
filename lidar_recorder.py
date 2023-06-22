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

import matplotlib.pyplot as plt
#Matplotlib is a library that allows us to plot data

import numpy as np
#Numpy is a library that allows us to do math with arrays

import datetime

#The Begining of the Code should be Static Methods
#Static Methods are functions that are not part of a class
#In this part is where we will place the Plotting Code

def plot_scan(ranges, angle_increment):

    # get the number of points in ranges
    n = len(ranges)

    # angles = []
    # for i in range(n):
    # angles.append(angle_increment * i)

    # create an array of LIDAR angle values corresponding to each range measurement
    thetas_rad = np.array([angle_increment * i for i in range(n)])
    #What this code does is that it creates an array of angles
    #angle increment * i is the angle of the ith measurement
    #Therefore if there are 10 measurements, the angles will be 0, 0.1, 0.2, 0.3, 0.4, 0.5, etc.
    thetas_deg = np.rad2deg(thetas_rad)
    #This converts the angles from radians to degrees
    
    #These are poloar coordinates
    x = ranges * np.cos(thetas_rad)
    y = ranges * np.sin(thetas_rad)

    f = plt.figure(1)
    a1 = plt.subplot(121)  # 121 -> 1 row, 2 columns, index 1
    a2 = plt.subplot(122)  # 122 -> 1 row, 2 columns, index 2

    a1.scatter(thetas_deg, ranges, cmap='jet', c=[i for i in range(n)])
    a1.set_xlabel("Scan Angle (degrees)")
    a1.set_ylabel("Range of Scan (m)")
    a1.grid()

    a2.scatter(x, y, cmap='jet', c=[i for i in range(n)])
    a2.set_aspect("equal")
    a2.set_xlabel("X (0.0 Starting Point) (m)")
    a2.set_ylabel("Y (0.0 Starting Point) (m)")
    a2.grid()

    plt.show()

def plot_scan_filtered(ranges_filtered, angle_increment_filtered):
    # get the number of points in ranges_filtered
    n_filtered = len(ranges_filtered)

    # create an array of LIDAR angle values corresponding to each range measurement
    thetas_rad_filtered = np.array([angle_increment_filtered * i for i in range(n_filtered)])
    thetas_deg_filtered = np.rad2deg(thetas_rad_filtered)

    # These are polar coordinates
    x_filtered = ranges_filtered * np.cos(thetas_rad_filtered)
    y_filtered = ranges_filtered * np.sin(thetas_rad_filtered)

    f_filtered = plt.figure(2)
    a1_filtered = plt.subplot(121)  # 121 -> 1 row, 2 columns, index 1
    a2_filtered = plt.subplot(122)  # 122 -> 1 row, 2 columns, index 2

    a1_filtered.scatter(thetas_deg_filtered, ranges_filtered, cmap='jet', c=[i for i in range(n_filtered)])
    a1_filtered.set_xlabel("Filtered Scan Angle (degrees)")
    a1_filtered.set_ylabel("Filtered Range (m)")
    a1_filtered.grid()

    a2_filtered.scatter(x_filtered, y_filtered, cmap='jet', c=[i for i in range(n_filtered)])
    a2_filtered.set_aspect("equal")
    a2_filtered.set_xlabel("X (0.0 Starting Point) (m)")
    a2_filtered.set_ylabel("Y (0.0 Starting Point) (m)")
    a2_filtered.grid()

    plt.show()

#Now This is the Instance Method portion of the code
#Instance Methods are functions that are part of a class
#We will create a class called Lidar_Recorder
class lidar_recorder:
    #Class is a blueprint for an object, in this case, Lidar_Recorder
    
    def __init__(self):
    #The __init__ function is called when an object of the class is created
    #Self refers to the current object
        
        rospy.init_node('lidar_recorder', anonymous=True)
        #rospy.init_node is a function that initializes the node
        #The node is called lidar_recorder
        #anonymous=True means that the node will have a unique name
        #Apllying that is a good practice
        #Now we will Subscribe to the /scan and /scan_filtered topics
        
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, callback=self.scan_callback, queue_size=10)
        #self.scan_subscriber is a variable that stores the subscriber
        #rospy.Subscriber is a function that subscribes to a topic
        #The topic is called /scan
        #The message type is LaserScan
        #Callback= is used to specify the callback function
        #The callback function is called every time a message is received from the topic
        #queue_size=10 is the max number of messages that can be stored in que
        #Its purpose is for us to be able to see the data from the topic
        
        #This Part is where we implement how to store the Storage
        self.latest_scan_msg = None
        #This will store the latest message from /scan
        self.new_scan_msg= False
        #This is used to determine if new message has been received


        self.scan_filtered_subscriber = rospy.Subscriber('/scan_filtered', LaserScan, callback=self.scan_filtered_callback, queue_size=10)
        #Same as above, but for /scan_filtered


        #This Part is where we implement how to store the Storage
        self.latest_scan_filtered_msg = None
        self.new_scan_filtered_msg= False
        #This is the same as above, but for /scan_filtered

    def scan_callback(self, msg: LaserScan):
        #This is the callback function for /scan
        #Self refers to the current object
        #msg: laserScan is the message that is received from the topic
        self.latest_scan_msg = msg
        #This stores the message in the latest_scan_msg variable
        self.new_scan = True
        #This sets the new_scan variable to True


    def scan_filtered_callback(self, msg: LaserScan):
        self.latest_scan_filtered_msg = msg
        self.new_scan_filtered = True


    def main(self):
    #This is the main function of the class
        rate=rospy.Rate(5)
        n_scan=0
        max_n_scan=30
        #n_scan is the number of scans that have been received
        #max_n_scan is the max number of scans that will be received
        m_filtered_scan=0
        max_m_filtered_scan=30
        #This is the same as above, but for /scan_filtered

        bool_scan_plot = True
        #This is used to determine if the plot for /scan should be displayed
        bool_scan_filtered_plot = True
        #Same as above, but for /scan_filtered

        #Implementing a timestamp in the file name
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        scan_filename = f"scan_data{timestamp}.csv"
        
        scan_filtered_filename = f"scan_filtered_data{timestamp}.csv"

        #Now to open the file
        
        with open(scan_filename, mode='w') as f:
            csv_writer = csv.writer(f, delimiter=',')
            #csv_writer() is a function that writes to a csv file
            #delimiter=',' means that the data will be separated by commas
            
            with open(scan_filtered_filename, mode='w') as f_filtered:
                csv_writer_filtered = csv.writer(f_filtered, delimiter=',')
               
               
               
                while not rospy.is_shutdown():
                #while not rospy.is_shutdown() means that the loop will run until control-c is pressed
                    if self.latest_scan_msg is not None:
                    #if self.new_scan is not None means that if a new message has been received
                    # get scan ranges and angle increment from LaserScan message
                        ranges = self.latest_scan_msg.ranges
                        angle_increment= self.latest_scan_msg.angle_increment
                    #self.latest_scan_msg.ranges is a list of the ranges from the LIDAR
                    
                        if bool_scan_plot:
                        #if bool scan plot is True, then the plot will be displayed
                            plot_scan(ranges, angle_increment)

                    #Now to write the data to the csv file
                        if self.new_scan:
                            csv_writer.writerow(ranges)
                                #csv_writer.writerow() is a function that writes a row to a csv file
                                #In this case, it writes the ranges to the csv file
                            n_scan += 1
                                #This increments the number of scans that have been received
                            if n_scan > max_n_scan:
                                self.new_scan = False
                                f.close()
                                return 
                                #This breaks the loop if the max number of scans has been reached
                
                    
                    if self.latest_scan_filtered_msg is not None:
                        ranges_filtered = self.latest_scan_filtered_msg.ranges
                        angle_increment_filtered = self.latest_scan_filtered_msg.angle_increment

                        if bool_scan_filtered_plot:
                            plot_scan_filtered(ranges_filtered, angle_increment_filtered)

                        if self.new_scan_filtered:
                            csv_writer_filtered.writerow(ranges_filtered)  # Write to scan_filtered.csv
                            m_filtered_scan += 1

                            if m_filtered_scan > max_m_filtered_scan:
                                self.new_scan_filtered = False
                                f_filtered.close()
                                return
                                
                            
                    rate.sleep()

if __name__ == '__main__':
    lidar_recorder().main()
