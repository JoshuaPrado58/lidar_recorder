#!usr/bin/env python3

#The above line is called a shebang. It tells the computer 
#what interpreter to use to run the file.

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import csv
import signal
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats
import sys
#This Code is to create a line of best fist
#Code meant for linear regression line

def read_csv():
    #def read csv is a function that reads the data from the csv file
    data = []
    #data = [] is used to store the data from the csv file
    with open('filename.csv', 'r') as f:
    #open the file in read mode
        csv_reader = csv.reader(f, delimiter=',')
        #csv_reader is a variable that reads the csv file
        #delimiter is a variable that seperates the data
        #This is utilized such as was used in the lidar_recorder.py script
        for row in csv_reader:
            #for loop that reads the csv file
            ranges = [float(value) for value in row]
    return np.array(data)
    #returns the data from the csv file as an array
    #dtype=float is used to convert the data from the csv file to a float

def main(filename):
    data = read_csv(filename)
    n_scans = data.shape[0]
    #n_scans is the number of scans
    #data.shape[0] is the number of rows in the csv file
    ranges= data[:, 0]
    #ranges is the first column of the csv file

    #Now we attempt to perform linear regression
    #Linear regression is a method used to find the best fit line
    #The best fit line is the line that best fits the data
    #The best fit line is the line that minimizes the error
    #The error is the distance between the data and the line
    #The error is also known as the residual

    x= np.arange(n_scans)
    #np.arange(n_scans) is an array of the number of scans
    slope, intercept, r_value, p_value, std_err = stats.linregress(x, ranges)
    regression_line = slope * x + intercept


    #Now we plot the data
    plt.plot(x, ranges, 'b.', label='Ranges')
    plt.plot(x, regression_line, 'r-', label='linear regression')
    plt.xlabel('Scan index')
    plt.ylabel('Range')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print('Please provide the filename of the CSV file.')
    else:
        filename = sys.argv[1]
        main(filename)
