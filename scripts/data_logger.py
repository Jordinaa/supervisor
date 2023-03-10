#!/usr/bin/env python3

from helper_functions import eulerToQuaternion, quaternionToEuler
import time 
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import PoseStamped

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

        self.last_timestamp = 0

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
        t = df['time']
        y1 = df['roll']
        y2 = df['pitch']
        y3 = df['yaw']

        # Plot data using Matplotlib
        plt.plot(t, y1, label='Roll')
        plt.plot(t, y2, label='Pitch')
        plt.plot(t, y3, label='Yaw')
        plt.legend()
        plt.title('Roll Pitch Yaw')
        plt.xlabel('Time')
        plt.ylabel('Angle in degrees')
        plt.show()

    def __del__(self):
        # Close the CSV file when the object is destroyed
        self.csv_file.close()
    
    # def live_plot(self):
    #     plt.plot(self.time_data, self.roll_data, label='Roll')
    #     plt.plot(self.time_data, self.pitch_data, label='Pitch')
    #     plt.plot(self.time_data, self.yaw_data, label='Yaw')
    #     plt.legend()
    #     plt.title('Roll Pitch Yaw')
    #     plt.xlabel('Time')
    #     plt.ylabel('Angle in degrees')

    #     return plt


if __name__ == '__main__': 
    rospy.init_node('data_logger_node')
    rospy.loginfo('data logger node started')
    datalogger = DataLogger()
    sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=datalogger.rpy_cb)
    rospy.spin()
    datalogger.__del__()
    datalogger.plot_csv()

