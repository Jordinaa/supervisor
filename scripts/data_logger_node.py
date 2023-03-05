#!/usr/bin/env python3

from helper_functions import eulerToQuaternion, quaternionToEuler

import math
import time 
import csv
import numpy as np
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
        self.csv_file = open("/home/taranto/catkin_ws/src/offboard_py/data/drone_data.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "roll", "pitch", "yaw"])

    def rpy_cb(self, msg: PoseStamped):
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        roll, pitch, yaw = quaternionToEuler(qx, qy, qz, qw)

        # converts from rad to degrees
        roll = np.rad2deg(roll)
        pitch = np.rad2deg(pitch)
        yaw = np.rad2deg(yaw)

        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.time_data.append(time.time() - self.start_time)
        self.roll_data.append(self.roll)
        self.pitch_data.append(self.pitch)
        self.yaw_data.append(self.yaw)
        self.csv_writer.writerow([self.time_data[-1], self.roll_data[-1], self.pitch_data[-1], self.yaw_data[-1]])

        rospy.loginfo(str(roll) + str(pitch) + str(yaw))

    def __del__(self):
        # Close the CSV file when the object is destroyed
        self.csv_file.close()


if __name__ == '__main__':
    # initialize data logger node 
    rospy.init_node('data_logger_node')
    rospy.loginfo('data logger node started')

    # inits datalogger class
    datalogger = DataLogger()

    # what the data is logging to terminal this is the roll pitch yaw 
    sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=datalogger.rpy_cb)
    datalogger.__del__
    rospy.spin()