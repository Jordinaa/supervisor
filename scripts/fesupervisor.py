#!/usr/bin/env python3

import signal
import sys
import argparse
import numpy as np
import csv
import datetime

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Header
# from mavros_msgs.msg import FlightTestInput
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget

import config
from helper import quaternionToEuler, eulerToQuaternion
from assessment import FlightEnvelopeAssessment
from supervisor.msg import DataLogger

class FlightEnvelopeSupervisor(FlightEnvelopeAssessment):
    """
    This class will check the current state of the drone and if it is out of bounds it will set it back to steady level flight.
    Once the flight envelope assessment is done. It will feed the bounds into this class which will setup the bounds.
    - need to fix the flag argument make it easier to check the bounds and set attitude back to 0
    """
    def __init__(self):
        super().__init__()
        self.rate = rospy.Rate(20)
        self.mode = 'OFFBOARD'

        self.roll = None
        self.pitch = None
        self.yaw = None
        # [self.qx, self.qy, self.qz, self.qw] = eulerToQuaternion(self.roll, self.pitch, self.yaw)
        # another try
        self.pb_roll = 0.0
        self.pb_pitch = 0.0
        self.pb_yaw = 0.0
        self.cb_true_v_sup = None
        self.cb_true_n_sup = None
        self.cb_predict_v_sup = None
        self.cb_predict_n_sup = None
        # FE Assessment is created these values will be set equal to whatever those bounds are
        self.flag = False

        subDataLogger = rospy.Subscriber('DataLogger', DataLogger, callback=self.data_logger_callback)
        # subFTI = rospy.Subscriber("mavros/flight_test_input", FlightTestInput, self.pti_cb)
        self.local_position_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.attitude_position_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)
        self.arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.setMode = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.static_bounds_vel, self.static_bounds_n = self.calc_load_factor_vs_velocity_static()
        self.num_lines_crossed = 0
        # CSV
        self.cb_time_list = []
        self.cb_true_n_list = []
        self.cb_true_v_list = []
        self.cb_predict_n_list = []
        self.cb_predict_v_list = []
        self.cb_key_event_time_list = []
        self.cb_key_event_list = []
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_path = f"/home/taranto/catkin_ws/src/supervisor/data/drone_data_{timestamp}.csv"
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "n_true", "true_velocity" "predicted_n", "predicted_velocity", "bounds_crossed"])

    def data_logger_callback(self, msg):
        self.cb_time = msg.time
        self.cb_true_n_sup = msg.true_n
        self.cb_true_v_sup = msg.true_velocity
        self.cb_predict_n_sup = msg.predicted_n
        self.cb_predict_v_sup = msg.predicted_velocity
        self.cb_key_event = msg.key_event

        self.cb_time_list.append(self.cb_time)
        self.cb_true_n_list.append(self.cb_true_n_sup)
        self.cb_true_v_list.append(self.cb_true_v_sup)
        self.cb_predict_n_list.append(self.cb_predict_n_sup)
        self.cb_predict_v_list.append(self.cb_predict_v_sup)
        self.cb_key_event_list.append(self.num_lines_crossed)
        self.csv_writer.writerow([self.cb_time_list[-1], self.cb_true_n_list[-1], self.cb_true_v_list[-1], self.cb_predict_n_list[-1], self.cb_predict_v_list[-1], self.num_lines_crossed])

    def shutdown(self, speed):
        rospy.loginfo("Shutting down Flight Envelope Supervisor...")
        self.set_velocity(speed, 0.0, 0.0)
        self.set_mode()
        self.arm_drone()
        # self.set_attitude()
        self.close_csv()
        rospy.signal_shutdown("Shutting down Flight Envelope Supervisor...")
        sys.exit(0)

    def set_velocity(self, x, y, z):
        velocity_command = Twist()
        velocity_command.linear.x = x
        velocity_command.linear.y = y
        velocity_command.linear.z = z

        velocity_publisher = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        velocity_publisher.publish(velocity_command)

    def send_straight_setpoint(self, speed):
        position_target = PositionTarget()
        position_target.header.stamp = rospy.Time.now()
        position_target.header.frame_id = "1"
        position_target.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        position_target.velocity.x = speed  # Set the desired x-axis velocity in meters per second
        position_target.velocity.y = 0.0
        position_target.velocity.z = 0.0
        self.local_position_pub.publish(position_target)

    def close_csv(self):
        self.csv_file.close()

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
        
    def pre_bake_commands(self):
        # for i in range(20):
        for i in range(100):   
            if(rospy.is_shutdown()):
                break
            self.flag = True
            self.set_attitude()
            self.rate.sleep()
            rospy.loginfo('prebake commands')
        self.flag = True

    def set_mode(self):
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

    def out_of_roll_bounds(self):
        roll_deg = np.rad2deg(self.assessment.roll)
        return (roll_deg > self.maxRoll) or (roll_deg < -self.maxRoll)
        
    def check_bounds(self, predicted_velocity, predicted_load_factor, velocities_lists, calc_load_factor_lists):
        num_lines_crossed = 0
        for calc_load_factor_list in calc_load_factor_lists:
            closest_index = np.argmin(np.abs(np.array(velocities_lists) - predicted_velocity))
            closest_load_factor = calc_load_factor_list[closest_index]
            # print(f"Predicted load factor: {predicted_load_factor:.4}, closest load factor: {closest_load_factor:.4}")
            
            if predicted_load_factor > closest_load_factor:
                num_lines_crossed += 1
    
        return num_lines_crossed

    def run(self):
        last_req = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # not sure how these are called back because the state callback is the only one that has self.current_state 
            current_state = self.current_state
            if current_state.mode != self.mode and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                self.set_mode()
                last_req = rospy.Time.now()

            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                self.arm_drone()
                last_req = rospy.Time.now()
                
            if current_state.mode == self.mode:
                self.num_lines_crossed = self.check_bounds(self.cb_predict_n_list[-1], self.cb_predict_v_list[-1], self.static_bounds_vel, self.static_bounds_n)
                print(f"while loop #: {self.num_lines_crossed}") 
                if self.num_lines_crossed == 1:
                    rospy.loginfo(f"Drone approaching flight envelope bound (level {self.num_lines_crossed}): Mild")
                elif self.num_lines_crossed == 2:
                    rospy.logwarn(f"ALERT: Drone is approaching flight envelope bound (level {self.num_lines_crossed}): Moderate")
                elif self.num_lines_crossed == 3:
                    rospy.logerr(f"CAUTION: Drone is breaching flight envelope bound (level {self.num_lines_crossed}): Severe")
                elif self.num_lines_crossed == 4:
                    rospy.logfatal(f"WARNING: Drone is nearing critical flight envelope bound (level {self.num_lines_crossed}): Critical")
                elif self.num_lines_crossed == 5:
                    rospy.logfatal(f"Drone has surpassed flight envelope (level {self.num_lines_crossed}): Fatal")

            last_req = rospy.Time.now()
            self.rate.sleep()

        self.close_csv()

def sigint_handler(sig, frame):
    rospy.loginfo("Stopping the script and making the drone fly in a straight line...")
    supervisor.shutdown(18.0)
    # rospy.sleep(1)
    sys.exit(0)

if __name__ == "__main__":

    rospy.init_node("Flight_Envelope_Supervisor")
    rate = rospy.Rate(20)

    supervisor = FlightEnvelopeSupervisor()
    
    signal.signal(signal.SIGINT, sigint_handler)

    supervisor.pre_bake_commands()
    supervisor.set_mode()
    supervisor.arm_drone()
    supervisor.set_attitude()
    supervisor.run()

