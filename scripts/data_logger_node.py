#!/usr/bin/env python3

import time 
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from mavros_msgs.msg import State, AttitudeTarget
from geometry_msgs.msg import PoseStamped


# callback function that prints out what you are recieving  

class DataLogger():
    """
    Logs data from the InformationNode and FlightEnvelopeSupervisor to a CSV file
    """
    def __init__(self, info_node, supervisor):
        self.info_node = info_node
        self.supervisor = supervisor

        self.time_data = []
        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []

        self.start_time = time.time()

        # Open the CSV file for writing
        self.csv_file = open("/home/taranto/catkin_ws/src/offboard_py/data/drone_data.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "roll", "pitch", "yaw"])

    def log_data(self):
        self.time_data.append(time.time() - self.start_time)
        self.roll_data.append(self.info_node.roll)
        self.pitch_data.append(self.info_node.pitch)
        self.yaw_data.append(self.info_node.yaw)

        # Write the roll, pitch, and yaw data to the CSV file
        self.csv_writer.writerow([self.time_data[-1], self.roll_data[-1], self.pitch_data[-1], self.yaw_data[-1]])

    def __del__(self):
        # Close the CSV file when the object is destroyed
        self.csv_file.close()





def dataLogger_cb(msg):
    rospy.loginfo(str(msg))

if __name__ == '__main__':

    rospy.init_node('data_logger_node')
    sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=dataLogger_cb)

    rospy.loginfo('data logger node started')
    rospy.spin()