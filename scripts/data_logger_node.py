#!/usr/bin/env python3

from helper_functions import eulerToQuaternion, quaternionToEuler

import math
import time 
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import PoseStamped


# callback function that prints out what you are recieving  

class DataLogger():
    """
    Logs data from the FlightEnvelopeSupervisor to a CSV file
    """
    def __init__(self):
        self.time_data = []
        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []
        self.start_time = time.time()
        self.roll = None
        self.pitch = None
        self.yaw = None

        # csv file
        self.csv_path = "/home/taranto/catkin_ws/src/offboard_py/data/drone_data.csv"
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "roll", "pitch", "yaw"])

    def rpy_cb(self, msg: PoseStamped):
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        roll, pitch, yaw = quaternionToEuler(qx, qy, qz, qw)

        # converts from rad to degrees
        roll = "{:.3f}".format(np.rad2deg(roll))
        pitch = "{:.3f}".format(np.rad2deg(pitch))
        yaw = "{:.3f}".format(np.rad2deg(yaw))

        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

        self.time_data.append(time.time() - self.start_time)
        self.roll_data.append(self.roll)
        self.pitch_data.append(self.pitch)
        self.yaw_data.append(self.yaw)
        self.csv_writer.writerow([self.time_data[-1], self.roll_data[-1], self.pitch_data[-1], self.yaw_data[-1]])

        rospy.loginfo(f" {str(roll)}, {str(pitch)}, {str(yaw)}")

    def plot_csv(self):
        df = pd.read_csv(self.csv_path)

        # Extract x and y data
        x = df['time']
        y1 = df['roll']
        y2 = df['pitch']
        y3 = df['yaw']

        # Plot data using Matplotlib
        plt.plot(x, y1, label='Roll')
        plt.plot(x, y2, label='Pitch')
        plt.plot(x, y3, label='Yaw')
        plt.legend()
        plt.title('Roll Pitch Yaw')
        plt.xlabel('Time')
        plt.ylabel('Angle in degrees')
        plt.show()

    def __del__(self):
        # Close the CSV file when the object is destroyed
        self.csv_file.close()

    # def plot_data(self):



if __name__ == '__main__':
    # initialize data logger node 
    rospy.init_node('data_logger_node')
    rospy.loginfo('data logger node started')

    # inits datalogger class
    datalogger = DataLogger()

    # what the data is logging to terminal this is the roll pitch yaw 
    sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=datalogger.rpy_cb)

    rospy.spin()
    datalogger.__del__()
    datalogger.plot_csv()

