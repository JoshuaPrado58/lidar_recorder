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

import sklearn.linear_model as lm



#This is where I will create the plot for the data

def plot_scan(ranges, angle_increment):
    angle_increment = 360 / num_ranges #This is the angle increnment in degrees
    angle_increment_rad = np.deg2rad(angle_increment)
    num_rows = len(ranges)  # Get the number of rows in the ranges array
    num_columns = len(ranges[0])  # Get the number of columns in the ranges array

    thetas_rad = np.array([angle_increment_rad * i for i in range(num_columns)])
    thetas_deg = np.array([angle_increment * i for i in range(num_columns)])

    f = plt.figure(1)
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

    #Now I Will be Recreating The Second Subplot, into a Second Fiugre
    #This is in Hopes I can use the data to create two linear regression line
    
    f2 = plt.figure(2)
    a3 = f2.add_subplot(111) 

    #Change Your Degrees Depending Where the object is located THIS IS A MUST
    limited_indices = np.where((thetas_deg >= 0) & (thetas_deg <= 60) | (thetas_deg >= 300) & (thetas_deg <= 360))
    #We use | to represent or, which allows us to have two conditions
    
    #We use | to represent or, which allows us to have two conditions
    limited_x = x[limited_indices]
    limited_y = y[limited_indices]

    a3.scatter(limited_x, limited_y, cmap='jet', c=thetas_deg[limited_indices])

    a3.set_aspect("equal")
    a3.set_xlabel("X (m)")
    a3.set_ylabel("Y (m)")
    a3.grid()

    # Fit linear regression lines for each angle range For The Respective Degrees
    limited_indices_1 = np.where((thetas_deg >= 0) & (thetas_deg <= 50))
    limited_indices_2 = np.where((thetas_deg >= 305) & (thetas_deg <= 355))

    limited_x_1 = x[limited_indices_1]
    limited_y_1 = y[limited_indices_1]

    limited_x_2 = x[limited_indices_2]
    limited_y_2 = y[limited_indices_2]

    regressor_1 = lm.LinearRegression()
    #regressor_1 is a variable that stores the linear regression line
    regressor_1.fit(limited_x_1.reshape(-1, 1), limited_y_1)
    #regressor_1.fit(limited_x_1.reshape(-1, 1), limited_y_1) is a function that fits the linear regression line

    regressor_2 = lm.LinearRegression()
    regressor_2.fit(limited_x_2.reshape(-1, 1), limited_y_2)

    # Generate predicted values for the linear regression lines
    predicted_y_1 = regressor_1.predict(limited_x_1.reshape(-1, 1))
    #regressor_1.predict(limited_x_1.reshape(-1, 1)) is a function that predicts the linear regression line
    predicted_y_2 = regressor_2.predict(limited_x_2.reshape(-1, 1))

    # Plot the linear regression lines

    a3.plot(limited_x_1, predicted_y_1, color='pink', label='Linear Regression 1')
    a3.plot(limited_x_2, predicted_y_2, color='black', label='Linear Regression 2')

    # Display linear regression equations
    equation_1 = f"Regression 1: y = {regressor_1.coef_[0]:.2f}x + {regressor_1.intercept_:.2f}"
    equation_2 = f"Regression 2: y = {regressor_2.coef_[0]:.2f}x + {regressor_2.intercept_:.2f}"
    print(f"First linear Regression Line : {equation_1}")
    print(f"Second linear Regression Line : {equation_2}")
    
    a3.text(1.0, 0.9, equation_1, transform=a3.transAxes, fontsize=10)
    a3.text(1.0, 0.85, equation_2, transform=a3.transAxes, fontsize=10)


    # This Part of the Code is making sure we can see the Intersectiion point
    #This is Due to the fact that the lines are not far enough to intersect
    # Extracting the slope and intercept of the first regression line
    
    #Use the link below to help for better help understanding intersection points
    #https://www.cuemath.com/geometry/intersection-of-two-lines/
    slope_1 = regressor_1.coef_[0]
    #This is the slope of the first regression line
    #regressor_1.coef_[0] is the slope of the first regression line
    
    intercept_1 = regressor_1.intercept_
    
    #This is the intercept of the first regression line
    # Extracting the slope and intercept of the second regression line
    
    slope_2 = regressor_2.coef_[0]
    intercept_2 = regressor_2.intercept_

    # Solving for the intersection point
    x_intersection = (intercept_2 - intercept_1) / (slope_1 - slope_2)
    y_intersection = slope_1 * x_intersection + intercept_1

    # Printing the intersection point
    print("Intersection Point:")
    print(f"x = {x_intersection:.4f}")
    print(f"y = {y_intersection:.4f}")
    
    a3.scatter(x_intersection, y_intersection, color='red', marker='o', label='Intersection')
    intersection_text = f"Intersection: ({x_intersection:.4f}, {y_intersection:.4f})"
    a3.text(1.0, 0.8, intersection_text, transform=a3.transAxes, fontsize=10)

    #In This part of the code I will be finding the angle created 
    #by the intersection point
    # Calculating the angle created by the linear regression lines at the point of intersection
    angle_intersection = np.abs(np.rad2deg(np.arctan2(slope_2 - slope_1, 1 + slope_1 * slope_2)))
    print(f"Intersection Angle: {angle_intersection:.4f} degrees")
    # Displaying the angle of the intersection lines
    intersection_angle_text = f"Intersection Angle: {angle_intersection:.4f} degrees"
    a3.text(1.0, 0.65, intersection_angle_text, transform=a3.transAxes, fontsize=10)

    a3.legend()
    
    plt.show()


#This Portion is where the file will be read, range and angle inrements of object
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






