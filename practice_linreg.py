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

def data_point_cheker(csv_file):
    with open(csv_file, 'r') as file:
        csv_reader = csv.reader(file)
        data_points = sum(1 for _ in csv_reader)
    return data_points

if __name__ == '__main__':
    # Check if the file name is provided as a command-line argument
    if len(sys.argv) < 2:
        print("Please provide a CSV file name.")
        sys.exit(1)

    # Extract the file name from command-line arguments
    csv_file = sys.argv[1]

    # Call the function to count data points
    num_data_points = data_point_cheker(csv_file)
    print(f"Number of data points: {num_data_points}")

