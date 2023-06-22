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

def read_csv_file(filename):
    #This function reads a csv file and returns an array of ranges
    ranges = []
    #ranges is an empty array
    with open(filename, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            ranges.extend(row)
    return np.array(ranges)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 script_name.py file_name.csv")
        sys.exit(1)

    file_name = sys.argv[1]
    ranges = read_csv_file(file_name)

    print(ranges)
