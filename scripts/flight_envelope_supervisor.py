#!/usr/bin/env python3
"""
 * File: offb_node.py
 * Stack and tested in Gazebo 9 SITL
"""
# helper files and datalogger function 
from helper_functions import quaternionToEuler, eulerToQuaternion
from data_logger import DataLogger

import argparse
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Header

class FlightEnvelopeSupervisor():
    """
    This class will check the current state of the drone and if it is out of bounds it will set it back to steady level flight.
    Once the flight envelope assessment is done. It will feed the bounds into this class which will setup the bounds.
    - need to fix the flag argument make it easier to check the bounds and set attitude back to 0
    """
    def __init__(self, roll, pitch, yaw=0.0):
        self.rate = rospy.Rate(20)
        self.mode = 'OFFBOARD'

        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        [self.qx, self.qy, self.qz, self.qw] = eulerToQuaternion(self.roll, self.pitch, self.yaw)
        # another try
        self.pb_roll = 0.0
        self.pb_pitch = 0.0
        self.pb_yaw = 0.0

        # FE Assessment is created these values will be set equal to whatever those bounds are
        self.flag = False
        self.maxRoll = 50

        self.local_position_topic = "mavros/setpoint_position/local"
        self.attitude_position_topic = "mavros/setpoint_raw/attitude"

        self.local_position_pub = rospy.Publisher(self.local_position_topic, PoseStamped, queue_size=10)
        self.attitude_position_pub = rospy.Publisher(self.attitude_position_topic, AttitudeTarget, queue_size=1)
  
        self.arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.setMode = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.info_node = InformationNode()
        self.info_node.init()

    def set_attitude(self):
        # Set the attitude of the drone by changing the roll angle
        attitude = AttitudeTarget()
        attitude.type_mask = AttitudeTarget.IGNORE_PITCH_RATE | AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_YAW_RATE
        attitude.header = Header()
        attitude.header.frame_id = "base_footprint"
        attitude.header.stamp = rospy.Time.now()

        if self.flag == True:
            attitude.orientation.x = 0.0
            attitude.orientation.y = 0.0
            attitude.orientation.z = 0.0
            attitude.orientation.w = 0.0

        else: 
            quaternion = eulerToQuaternion(self.roll, self.pitch, self.yaw)
            attitude.thrust = 0.5
            attitude.orientation.x = quaternion[0]
            attitude.orientation.y = quaternion[1]
            attitude.orientation.z = quaternion[2]
            attitude.orientation.w = quaternion[3]

        self.attitude_position_pub.publish(attitude)
        
    def pre_bake_commanders(self):
        for i in range(100):   
            if(rospy.is_shutdown()):
                break
            self.flag = True
            self.set_attitude()
            self.rate.sleep()
            rospy.loginfo('prebake commands')
        self.flag = True

    def set_mode(self):
        # Set the flight mode of the drone
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = self.mode

        try:
            rospy.wait_for_service('mavros/set_mode', timeout=5)
            set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
            set_mode(offb_set_mode)
            rospy.loginfo(f"{self.mode} mode enabled")
        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call failed: {e}")

    def arm_drone(self):
        # Arm the drone
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        if self.arm.call(arm_cmd).success == True:
            rospy.loginfo("Vehicle armed")

    # def out_of_roll_bounds(self):
    #     if np.rad2deg(self.info_node.roll) > self.maxRoll or np.rad2deg(self.info_node.roll) < -self.maxRoll:
    #         rospy.logwarn('Out of bounds')
    #         return True
    #     return False

    def out_of_roll_bounds(self):
        roll_deg = np.rad2deg(self.info_node.roll)
        return (roll_deg > self.maxRoll) or (roll_deg < -self.maxRoll)

    def run(self):
        last_req = rospy.Time.now()

        while not rospy.is_shutdown():
            # not sure how these are called back because the state callback is the only one that has self.current_state 
            current_state = self.info_node.current_state
            if current_state.mode != self.mode and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                self.set_mode()
                last_req = rospy.Time.now()

            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                self.arm_drone()
                last_req = rospy.Time.now()
    
            if current_state.mode == self.mode:
                if self.out_of_roll_bounds() == True:
                    while np.rad2deg(self.info_node.roll) > 1:
                        self.flag = True
                        self.set_attitude()
                        self.rate.sleep()

                else:
                    self.flag = False
                    self.set_attitude()

            last_req = rospy.Time.now()
            self.rate.sleep()


class InformationNode():
    """
    This class creates subscribers to local position and state,
    and has position callback functions
    """
    def init(self):
        # self.mode = None
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.state_topic = "mavros/state"
        self.local_position_topic = "mavros/local_position/pose"
        self.state_sub = rospy.Subscriber(self.state_topic, State, callback=self.state_cb)
        self.position_sub = rospy.Subscriber(self.local_position_topic, PoseStamped, callback=self.position_cb)
        print("Information Node initialized")

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

 
if __name__ == "__main__":

    # Parse command line arguments for roll and pitch
    parser = argparse.ArgumentParser()
    parser.add_argument("roll", type=float, help="Desired roll angle in degrees")
    parser.add_argument("pitch", type=float, help="Desired pitch angle in degrees")
    args = parser.parse_args()

    # Initialize ROS node
    rospy.init_node("Flight Envelope Supervisor")
    rate = rospy.Rate(20)

    # Here you will initalize the FEA and feed those return values into the FES class
    # which will set the bounds for the supervisor

    supervisor = FlightEnvelopeSupervisor(args.roll, args.pitch)
    supervisor.pre_bake_commanders()
    supervisor.set_mode()
    supervisor.arm_drone()
    supervisor.set_attitude()
    supervisor.run()

