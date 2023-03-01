#!/usr/bin/env python3
"""
 * File: offb_node.py
 * Stack and tested in Gazebo 9 SITL
"""
from xmlrpc.client import Boolean
import math 
import time 
import csv
import numpy as np
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
    this class will supervise the drone 
    will also control it
    """
    def __init__(self, x, y, z, roll = 0.0, pitch = 0.0, yaw = 0.0):
        self.x = x
        self.y = y
        self.z = z

        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw 
        self.coords = [None, None, None]

        self.local_position_topic = "mavros/setpoint_position/local"
        self.attitude_position_roll_topic = "mavros/setpoint_raw/attitude"
        self.attitude_position_pitch_topic = "mavros/setpoint_raw/attitude"
        self.local_position_pub = rospy.Publisher(self.local_position_topic, PoseStamped, queue_size = 10)
        self.attitude_position_roll_pub = rospy.Publisher(self.attitude_position_roll_topic, AttitudeTarget, queue_size = 1)
        self.attitude_position_pitch_pub = rospy.Publisher(self.attitude_position_pitch_topic, AttitudeTarget, queue_size = 1)

    def set_pose(self):
        pose = PoseStamped()

        [self.qx, self.qy, self.qz, self.qw] = eulerToQuaternion(self.roll, self.pitch, yaw = 0.0)
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        pose.pose.orientation.x = self.qx
        pose.pose.orientation.y = self.qy
        pose.pose.orientation.z = self.qz
        pose.pose.orientation.w = self.qw

        # Publish pose to local position topic
        self.local_position_pub.publish(pose)

class InformationNode():
    """
    this class creates subscribers to local position and state
    has position callback functions
    """
    def __init__(self):
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.current_state = State()
        self.state_topic = "mavros/state"
        self.local_position_topic = "mavros/local_position/pose"
        self.state_sub = rospy.Subscriber(self.state_topic, State, callback = self.state_cb)
        self.position_sub = rospy.Subscriber(self.local_position_topic, PoseStamped, callback = self.position_cb)

    def state_cb(self, msg):
        self.current_state = msg

    def position_cb(self, msg):
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.roll, self.pitch, self.yaw = quaternionToEuler(qx ,qy, qz, qw)


# class DataLogging():


if __name__ == "__main__":

    rospy.init_node("offb_node_py")
    print("starting")

    current_state = State()
    pose = PoseStamped()

    # subscribers
    infoNode = InformationNode()
    infoNode.state_sub
    infoNode.position_sub

    # publishers
    supervisor = FlightEnvelopeSupervisor(0, 0, 50, 0, 0)
    supervisor.local_position_pub
    supervisor.attitude_position_roll_pub
    supervisor.attitude_position_pitch_pub
    
    # services
    rospy.wait_for_service("/mavros/cmd/arming")
    arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
    rospy.wait_for_service("/mavros/set_mode")
    setMode = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    # fix this #######################################
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    rate = rospy.Rate(20)

    last_req = rospy.Time.now()
    
    while not rospy.is_shutdown():

        current_state = infoNode.current_state  # update current_state
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(setMode(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()

        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arm(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()

        if current_state.mode == "OFFBOARD":
            print('WORKING')
            supervisor.set_pose()

        rate.sleep()
