#!/usr/bin/env python3

import math
import time 
import csv
import numpy as np
import rospy
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

def quaternionToEuler(x:float, y:float, z:float, w:float) -> tuple:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians



def dataLogger_cb(msg: PoseStamped):
    qx = msg.pose.orientation.x
    qy = msg.pose.orientation.y
    qz = msg.pose.orientation.z
    qw = msg.pose.orientation.w
    roll, pitch, yaw = quaternionToEuler(qx, qy, qz, qw)

    roll = np.rad2deg(roll)
    pitch = np.rad2deg(pitch)
    yaw = np.rad2deg(yaw)
    
    rospy.loginfo(str(pitch) + ", " + str(pitch) + ", " + str(yaw))

if __name__ == '__main__':

    rospy.init_node('data_logger_node')
    sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=dataLogger_cb)

    rospy.loginfo('data logger node started')
    rospy.spin()