#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import numpy as np
import sys

def load_data_from_csv(filename):
    scan_data = []
    scan_filtered_data = []

    with open(filename, 'r') as file:
        csv_reader = csv.reader(file)

        for row in csv_reader:
            if row[0] == 'scan':
                scan_data.extend(map(float, row[1:]))
            elif row[0] == 'scan_filtered':
                scan_filtered_data.extend(map(float, row[1:]))

    return scan_data, scan_filtered_data

def plot_scatter(scan_data, scan_filtered_data):
    plt.scatter(range(len(scan_data)), scan_data, label='Scan')
    plt.scatter(range(len(scan_filtered_data)), scan_filtered_data, label='Scan Filtered')
    plt.xlabel('Data Point')
    plt.ylabel('Range')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 lid_rec_plot_converter.py <csv_filename>")
        sys.exit(1)

    filename = sys.argv[1]
    scan_data, scan_filtered_data = load_data_from_csv(filename)
    plot_scatter(scan_data, scan_filtered_data)


