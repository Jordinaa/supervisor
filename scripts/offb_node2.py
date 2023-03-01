#!/usr/bin/env python3
"""
 * File: offb_node.py
 * Stack and tested in Gazebo 9 SITL
"""
# this has pitch command
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import math 
import numpy as np
from std_msgs.msg import Header

current_state = State()
global_yaw = 0.0

def euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple:
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

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
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

    return [w, x, y, z]

# takes input for attitude and returns an attitude target message that publishes
# to setpoint_raw/attitude topic
# creates an attitude target message sets type mask to ignore rpy and yaw rates
# sets frame id to base_footprint and sets the header stamp to current time
# calculates current yaw using global yaw and conver
def send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                          yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5, body_roll_rate=0.0, body_pitch_rate=0.0):

    #https://github.com/mavlink/mavros/issues/1356
    attitude_msg = AttitudeTarget()
    attitude_msg.type_mask = AttitudeTarget.IGNORE_PITCH_RATE| AttitudeTarget.IGNORE_ROLL_RATE| AttitudeTarget.IGNORE_YAW_RATE#4#4#0b00000000 
    attitude_msg.header = Header()
    attitude_msg.header.frame_id = "base_footprint"
    attitude_msg.header.stamp = rospy.Time.now()

    yaw_angle = global_yaw
    quaternion = to_quaternion(roll_angle, pitch_angle, yaw_angle)

    # r,p,y = euler_from_quaternion(quaternion[1], quaternion[2], quaternion[3], quaternion[0])

    # print("roll command is", np.rad2deg(r))
    # print("pitch command is", np.rad2deg(p))

    attitude_msg.orientation.x = quaternion[1]
    attitude_msg.orientation.y = quaternion[2]
    attitude_msg.orientation.z = quaternion[3]
    attitude_msg.orientation.w = quaternion[0]
    
    # attitude_msg.body_rate.x = body_pitch_rate
    # attitude_msg.body_rate.y = body_roll_rate
    # attitude_msg.body_rate.z = yaw_rate

    attitude_msg.thrust = 0.5    
    # print(attitude_msg)

    return attitude_msg

# callback functions that get called when the drones state or position changes
# they update the current state and global yaw angle variables 
def state_cb(msg):
    global current_state
    current_state = msg

def position_cb(msg):
    global global_yaw 
    qx = msg.pose.orientation.x
    qy = msg.pose.orientation.y
    qz = msg.pose.orientation.z
    qw = msg.pose.orientation.w

    roll, pitch, global_yaw = euler_from_quaternion(qx ,qy, qz, qw)
    # print("roll", np.rad2deg(roll))
    # print("pitch", np.rad2deg(pitch))



if __name__ == "__main__":
    rollCommand = 0
    pitchCommand = 0
    rospy.init_node("offb_node_py")

    # subscribes to mavros/state, state 
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    # publishing to the mavros/setpoint_position/local topic, the messae type is State, size of outgoing message
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    position_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=position_cb)

    attitude_pos_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)
    
    attitude_pos_pitch_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)


    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        print('hm')
        rate.sleep()

    pose = PoseStamped()
    [qw,qx,qy,qz] = to_quaternion(roll=rollCommand, pitch=pitchCommand, yaw=0)
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 50
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    
    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break
        # roll send and publish
        attitude_roll = send_attitude_target(roll_angle=rollCommand)
        attitude_pos_pub.publish(attitude_roll)
        # pitch send and pubish
        attitude_pitch = send_attitude_target(pitch_angle=pitchCommand)
        attitude_pos_pitch_pub.publish(attitude_pitch)

        # local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
        
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
        
                last_req = rospy.Time.now()


        if current_state.mode == "OFFBOARD":
            print("sending attittude")
            attitude_msg = send_attitude_target(roll_angle=rollCommand)
            attitude_pos_pub.publish(attitude_msg)

            attitude_pitch_msg = send_attitude_target(pitch_angle=pitchCommand)
            attitude_pos_pub.publish(attitude_pitch)
            
        else:
            local_pos_pub.publish(pose)

        rate.sleep()
