#!usr/bin/env python3
#The above line is called a shebang

import rospy
#By importing rospy, we accesss all the entire ROS Library

from sensor_msgs.msg import LaserScan

from std_msgs.msg import String

import csv

import sys

import signal

import matplotlib.pyplot as plt

import numpy as np

import datetime



#This is where I will create the plot for the data

def plot_scan(ranges, angle_increment):
    angle_increment = 360 / num_ranges
    angle_increment_rad = np.deg2rad(angle_increment)
    num_rows = len(ranges)  # Get the number of rows in the ranges array
    num_columns = len(ranges[0])  # Get the number of columns in the ranges array

    thetas_rad = np.array([angle_increment_rad * i for i in range(num_columns)])
    thetas_deg = np.array([angle_increment * i for i in range(num_columns)])

    f = plt.figure()
    a1 = plt.subplot(121)  # 121 -> 1 row, 2 columns, index 1
    a2 = plt.subplot(122)  # 122 -> 1 row, 2 columns, index 2

    # Iterate over each row of the ranges array
    for row in ranges:
        x = row * np.cos(thetas_rad)
        y = row * np.sin(thetas_rad)

        a1.scatter(thetas_deg, row, cmap='jet', c=[i for i in range(num_columns)])
        a2.scatter(x, y, cmap='jet', c=[i for i in range(num_columns)])

    a1.set_xlabel("Scan Angle (degrees)")
    a1.set_ylabel("Range (m)")
    a1.grid()

    a2.set_aspect("equal")
    a2.set_xlabel("X (m)")
    a2.set_ylabel("Y (m)")
    a2.grid()

    plt.show()



def range_counter(csv_file):
   #This is function is used to count the number of ranges in the csv file
   #It is used to be able to make sure that the file is being read properly
   #Due to being in correlation with the lidar_recorder.py script
        with open(csv_file, mode='r') as f:
       #open (csv_file, mode='r') is used to open the csv file in read mode
       #as f is used to store the data from the csv file
            csv_reader = csv.reader(f)
       #csv_reader is a variable that reads the csv file
       #csv.reader(f) is a function that reads the csv file
            ranges = list(csv_reader)
       #ranges is a variable that stores the data from the csv file
       #list(csv_reader) is a function that converts the data from the csv file to a list
            num_ranges = len(ranges[0])
       #num_ranges is a variable that stores the number of ranges in the csv file
       #len(ranges[0]) is a function that counts the number of ranges in the csv file
            return num_ranges


if __name__ == '__main__':
    # Check if the file name is provided as a command-line argument
    if len(sys.argv) < 2:
        print("Please provide a CSV file name.")

    else:
        # Extract the file name from command-line arguments
        csv_file = sys.argv[1]

        # Call the function to count data points
        num_ranges = range_counter(csv_file)
        print(f"Number of ranges: {num_ranges}")
        angle_increment = 360/num_ranges
        ranges = np.loadtxt(csv_file, delimiter=',')

     # Call the function to plot the scan
        plot_scan(ranges, angle_increment)





