#!/usr/bin/env python3
"""
 * File: offb_node.py
 * Stack and tested in Gazebo 9 SITL
"""
# helper files and datalogger function 
from helper_functions import quaternionToEuler, eulerToQuaternion
from data_logger_node import DataLogger

import argparse
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Header


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
                        rate.sleep()
                        # DONE = True
                else:
                    self.set_attitude(rollcmd, pitchcmd)
                    # if DONE == True:
                        # self.set_attitude()  
                    # else:
                        # self.set_attitude(rollcmd, pitchcmd)
                # rospy.loginfo("beep boop")
            last_req = rospy.Time.now()
            rate.sleep()

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
        print("infonode initialized")

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
    supervisor.run(rate, args.roll, args.pitch)

    