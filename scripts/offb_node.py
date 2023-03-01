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
from mpl_toolkits.mplot3d import Axes3D
from xmlrpc.client import Boolean
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
    def __init__(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.coords = [None, None, None]
        [self.qx, self.qy, self.qz, self.qw] = eulerToQuaternion(self.roll, self.pitch, self.yaw)

        self.local_position_topic = "mavros/setpoint_position/local"
        self.attitude_position_roll_topic = "mavros/setpoint_raw/attitude"
        self.attitude_position_pitch_topic = "mavros/setpoint_raw/attitude"
        self.local_position_pub = rospy.Publisher(self.local_position_topic, PoseStamped, queue_size=10)
        self.attitude_position_roll_pub = rospy.Publisher(self.attitude_position_roll_topic, AttitudeTarget, queue_size=1)
        self.attitude_position_pitch_pub = rospy.Publisher(self.attitude_position_pitch_topic, AttitudeTarget, queue_size=1)

        self.arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.setMode = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # Create an instance of the InformationNode class
        self.info_node = InformationNode()
        # Initialize the information node
        self.info_node.init()
        # Set initial position and orientation of the drone
        pose = PoseStamped()
        
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        pose.pose.orientation.x = self.qx
        pose.pose.orientation.y = self.qy
        pose.pose.orientation.z = self.qz
        pose.pose.orientation.w = self.qw
        self.local_position_pub.publish(pose)

    def set_attitude_roll(self, roll):
        # Set the attitude of the drone by changing the roll angle
        attitude = AttitudeTarget()
        attitude.type_mask = AttitudeTarget.IGNORE_PITCH_RATE | AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_YAW_RATE#4#4#0b00000000 
        attitude.header = Header()
        attitude.header.frame_id = "base_footprint"
        attitude.header.stamp = rospy.Time.now()

        attitude.thrust = 0.5
        attitude.orientation.x = self.qx
        attitude.orientation.y = self.qy
        attitude.orientation.z = self.qz # change
        attitude.orientation.w = self.qw # change

        self.attitude_position_roll_pub.publish(attitude)


    # def set_attitude_pitch(self, pitch):
    #     # Set the attitude of the drone by changing the pitch angle
    #     attitude = AttitudeTarget()
    #     attitude.type_mask = AttitudeTarget.IGNORE_PITCH_RATE | AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_YAW_RATE#4#4#0b00000000 
    #     # attitude.body_rate.x = 0.0
    #     # attitude.body_rate.y = 0.0
    #     # attitude.body_rate.z = 0.0
    #     attitude.thrust = 0.5
    #     attitude.orientation.x = 0.0
    #     attitude.orientation.y = self.qy #change
    #     attitude.orientation.z = 0.0
    #     attitude.orientation.w = self.qw # change
    #     self.attitude_position_pitch_pub.publish(attitude)

    def pre_bake_waypoints(self, rate, rollcomm):
        for i in range(100):   
            if(rospy.is_shutdown()):
                break
            # roll send and publish
            self.set_attitude_roll(roll=rollcomm)
            # pitch send and pubish
            # local_pos_pub.publish(pose)
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

    def run(self, rate):
        # Main loop to supervise and control the drone
        rate = rate
        last_req = rospy.Time.now()
        while not rospy.is_shutdown():
            # Check the current state of the drone

            # self.info_node.update()
            current_state = self.info_node.current_state

            # current_state = rospy.wait_for_message("mavros/state", State)
            if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                # Set the drone to OFFBOARD mode
                print("case 1")
                self.set_mode("OFFBOARD")
                last_req = rospy.Time.now()

            # if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            #     print("case 2")
            #     # Arm the drone
            #     self.arm_drone()
            #     last_req = rospy.Time.now()
    
            if current_state.mode == "OFFBOARD":
                print("case 3")
                # Set the attitude of the drone
                self.set_attitude_roll(self.roll)
                # self.set_attitude_pitch(self.pitch)
                rospy.loginfo("rolling and pitching")

            # if (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            #     last_req = rospy.Time.now()
            print("hello world")
            last_req = rospy.Time.now()
            rate.sleep()

class DataLogger():
    """
    Logs and plots data from the InformationNode and FlightEnvelopeSupervisor
    """
    def __init__(self, info_node, supervisor):
        self.info_node = info_node
        self.supervisor = supervisor

        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []
        self.x_data = []
        self.y_data = []
        self.z_data = []

        self.fig, self.axs = plt.subplots(2, 3, figsize=(12, 8))
        self.fig.suptitle("Drone Data Logger")

        # Initialize the subplots
        self.axs[0, 0].set_title("Roll")
        self.axs[0, 0].set_xlabel("Time (s)")
        self.axs[0, 0].set_ylabel("Roll (deg)")

        self.axs[0, 1].set_title("Pitch")
        self.axs[0, 1].set_xlabel("Time (s)")
        self.axs[0, 1].set_ylabel("Pitch (deg)")

        self.axs[0, 2].set_title("Yaw")
        self.axs[0, 2].set_xlabel("Time (s)")
        self.axs[0, 2].set_ylabel("Yaw (deg)")

        self.axs[1, 0].set_title("X Position")
        self.axs[1, 0].set_xlabel("Time (s)")
        self.axs[1, 0].set_ylabel("X Position (m)")

        self.axs[1, 1].set_title("Y Position")
        self.axs[1, 1].set_xlabel("Time (s)")
        self.axs[1, 1].set_ylabel("Y Position (m)")

        self.axs[1, 2].set_title("Z Position")
        self.axs[1, 2].set_xlabel("Time (s)")
        self.axs[1, 2].set_ylabel("Z Position (m)")

        # Initialize the plot lines
        self.roll_line, = self.axs[0, 0].plot([], [], 'b-')
        self.pitch_line, = self.axs[0, 1].plot([], [], 'b-')
        self.yaw_line, = self.axs[0, 2].plot([], [], 'b-')
        self.x_line, = self.axs[1, 0].plot([], [], 'b-')
        self.y_line, = self.axs[1, 1].plot([], [], 'b-')
        self.z_line, = self.axs[1, 2].plot([], [], 'b-')

        # Set the axes limits
        self.axs[0, 0].set_ylim(-180, 180)
        self.axs[0, 1].set_ylim(-180, 180)
        self.axs[0, 2].set_ylim(-180, 180)
        self.axs[1, 0].set_ylim(-50, 50)
        self.axs[1, 1].set_ylim(-50, 50)
        self.axs[1, 2].set_ylim(0, 100)

        # Initialize the time
        self.start_time = time.time()

    def update(self):
        # Log the latest data
        self.log_data()

        # Update the live plots
        for i in range(len(self.plots)):
            self.plots[i].setData(self.data[:, 0], self.data[:, i+1])
        
        # Update the GUI
        QtWidgets.QApplication.processEvents()

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
    supervisor = FlightEnvelopeSupervisor(0, 0, 50, 0, 0)
    # Arm the drone
    supervisor.pre_bake_waypoints(rate, 0)
    supervisor.set_mode("OFFBOARD")
    supervisor.arm_drone()
    # Set drone to OFFBOARD mode
    # Set the attitude of the drone based on input arguments
    # print(f' main function roll {args.roll}')
    # print(f' main function pitch {args.pitch}')
    supervisor.set_attitude_roll(args.roll)
    # supervisor.set_attitude_pitch(args.pitch)
    # Run the main loop
    supervisor.run(rate)
