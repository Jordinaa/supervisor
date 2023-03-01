#!/usr/bin/env python3
"""
 * File: offb_node.py
 * Stack and tested in Gazebo 9 SITL
"""
import math 
import time 
import csv
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from PyQt5 import QtWidgets, QtCore

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Header

def eulerToQuaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [x, y, z, w]

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


class FlightEnvelopeSupervisor():
    """
    This class will supervise and control the drone
    """
    def __init__(self, roll, pitch, yaw=0.0):
        self.x = None
        self.y = None
        self.z = None
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.coords = [None, None, None]
        [self.qx, self.qy, self.qz, self.qw] = eulerToQuaternion(self.roll, self.pitch, self.yaw)

        self.local_position_topic = "mavros/setpoint_position/local"
        self.attitude_position_topic = "mavros/setpoint_raw/attitude"
        self.attitude_position_pitch_topic = "mavros/setpoint_raw/attitude"
        self.local_position_pub = rospy.Publisher(self.local_position_topic, PoseStamped, queue_size=10)
        self.attitude_position_pub = rospy.Publisher(self.attitude_position_topic, AttitudeTarget, queue_size=1)

        self.arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.setMode = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # Create an instance of the InformationNode class
        self.info_node = InformationNode()
        # Initialize the information node
        self.info_node.init()
        self.data_logger = DataLogger(self.info_node, self)

    def set_attitude(self, roll=0.0, pitch=0.0, yaw=0.0):
        # Set the attitude of the drone by changing the roll angle
        attitude = AttitudeTarget()
        attitude.type_mask = AttitudeTarget.IGNORE_PITCH_RATE | AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_YAW_RATE
        attitude.header = Header()
        attitude.header.frame_id = "base_footprint"
        attitude.header.stamp = rospy.Time.now()
        quaternion = eulerToQuaternion(roll, pitch, yaw)
        # print(roll)
        attitude.thrust = 0.5
        attitude.orientation.x = quaternion[0]
        attitude.orientation.y = quaternion[1]
        attitude.orientation.z = quaternion[2]
        attitude.orientation.w = quaternion[3]
        self.attitude_position_pub.publish(attitude)

    def pre_bake_commanders(self, rate, rollcomm, pitchcomm):
        for i in range(100):   
            if(rospy.is_shutdown()):
                break
            self.set_attitude(rollcomm, pitchcomm)
            rate.sleep()

    def set_mode(self, mode):
        # Set the flight mode of the drone
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = mode

        if self.setMode.call(offb_set_mode).mode_sent == True:
            rospy.loginfo(f"{mode} mode enabled")

    def arm_drone(self):
        # Arm the drone
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        if self.arm.call(arm_cmd).success == True:
            rospy.loginfo("Vehicle armed")

    def out_of_roll_bounds(self, roll):
        maxRoll = 50
        # roll is in degrees
        if np.rad2deg(roll) > maxRoll or np.rad2deg(roll) < -maxRoll:
            # rospy.log
            print('out of bounds')
            return True
        return False

    def run(self, rate, rollcmd, pitchcmd):
        # Main loop to supervise and control the drone

        # DONE = False
        rate = rate
        last_req = rospy.Time.now()
        while not rospy.is_shutdown():
            # Check the current state of the drone

            # self.info_node.update()
            current_state = self.info_node.current_state

            if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                # Set the drone to OFFBOARD mode
                self.set_mode("OFFBOARD")
                last_req = rospy.Time.now()

            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                # Arm the drone
                self.arm_drone()
                last_req = rospy.Time.now()
    
    
            if current_state.mode == "OFFBOARD":
                # set attitude
                if self.out_of_roll_bounds(self.info_node.roll) == True:
                    while np.rad2deg(self.info_node.roll) > 1: #and np.rad2deg(self.info_node.roll) < -1:
                        self.set_attitude()  
                        self.data_logger.log_data()                     
                        rate.sleep()
                        # DONE = True
                else:
                    self.set_attitude(rollcmd, pitchcmd)
                    # if DONE == True:
                        # self.set_attitude()  
                    # else:
                        # self.set_attitude(rollcmd, pitchcmd)
                # rospy.loginfo("beep boop")
                self.data_logger.log_data()




            last_req = rospy.Time.now()
            rate.sleep()

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
    


class InformationNode():
    """
    This class creates subscribers to local position and state,
    and has position callback functions
    """
    def init(self):
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.state_topic = "mavros/state"
        self.local_position_topic = "mavros/local_position/pose"
        self.state_sub = rospy.Subscriber(self.state_topic, State, callback=self.state_cb)
        self.position_sub = rospy.Subscriber(self.local_position_topic, PoseStamped, callback=self.position_cb)
        print("InformationNode initialized")

    def state_cb(self, msg):
        self.current_state = msg
        # print('state callback func')

    def position_cb(self, msg):
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.roll, self.pitch, self.yaw = quaternionToEuler(qx, qy, qz, qw)
        # print('position call back func')

    # def update(self):
    #         rospy.loginfo(f"Current roll: {self.roll}, pitch: {self.pitch}, yaw: {self.yaw}")


if __name__ == "__main__":
    # Parse command line arguments for roll and pitch
    parser = argparse.ArgumentParser()
    parser.add_argument("roll", type=float, help="Desired roll angle in degrees")
    parser.add_argument("pitch", type=float, help="Desired pitch angle in degrees")
    args = parser.parse_args()

    # Initialize ROS node
    rospy.init_node("offb_node_py")
    rate = rospy.Rate(20)

    # Create instance of FlightEnvelopeSupervisor class
    supervisor = FlightEnvelopeSupervisor(args.roll, args.pitch)
    supervisor.pre_bake_commanders(rate, args.roll, args.pitch)
    supervisor.set_mode("OFFBOARD")
    supervisor.arm_drone()
    supervisor.set_attitude(args.roll, args.pitch)
    data_logger = DataLogger(supervisor.info_node, supervisor)
    supervisor.run(rate, args.roll, args.pitch)

