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
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped

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
        self.flag = False

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        [self.qx, self.qy, self.qz, self.qw] = eulerToQuaternion(self.roll, self.pitch, self.yaw)
        self.pb_roll = 0.0
        self.pb_pitch = 0.0
        self.pb_yaw = 0.0
        self.cb_true_v_sup = 0.0
        self.cb_true_n_sup = 0.0
        self.cb_predict_v_sup = 0.0
        self.cb_predict_n_sup = 0.0

        # subFTI = rospy.Subscriber("mavros/flight_test_input", FlightTestInput, self.pti_cb)
        subDataLogger = rospy.Subscriber('DataLogger', DataLogger, callback=self.data_logger_callback)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        # rospy.init_node('drone_velocity_control', anonymous=True)
        self.local_position_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.attitude_position_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)
        self.arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.setMode = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.local_position_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=self.local_position_callback)

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

    def local_position_callback(self, msg):
        self.local_position = msg

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

    def close_csv(self):
        self.csv_file.close()

##############
    def shutdown(self, speed):
        rospy.loginfo("Shutting down Flight Envelope Supervisor...")
        self.pre_bake_commands()
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

    def set_straight(self, speed):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = rospy.Time.now()
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.twist.linear.x = speed
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = 0.0

        velocity_publisher = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)
        velocity_publisher.publish(twist_stamped)
 
    def takeoff(self, target_altitude):
        """
        Make the drone take off to the target altitude.
        """
        rospy.loginfo("Taking off...")
        takeoff_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        takeoff_pose = PoseStamped()
        takeoff_pose.pose.position.x = 0
        takeoff_pose.pose.position.y = 0
        takeoff_pose.pose.position.z = target_altitude
        reached_target_altitude = False
        while not reached_target_altitude and not rospy.is_shutdown():
            local_position = self.local_position.pose.position
            altitude_error = abs(target_altitude - local_position.z)
            if altitude_error < 0.5:  # Acceptable altitude error, change this value if necessary
                reached_target_altitude = True
                rospy.loginfo("Reached target altitude")
            takeoff_pub.publish(takeoff_pose)
            self.rate.sleep()
        
    def steady_level_flight(self, speed=18.0):
        velocity_msg = TwistStamped()
        velocity_msg.header = Header(frame_id='base_link')
        velocity_msg.header.stamp = rospy.Time.now()
        # Set the linear velocities (in meters per second)
        if self.flag == True:
            velocity_msg.twist.linear.x = speed
            velocity_msg.twist.linear.y = 0.0
            velocity_msg.twist.linear.z = 0.0
            # Set the angular rates (in radians per second)
            velocity_msg.twist.angular.x = 0.0
            velocity_msg.twist.angular.y = 0.0
            velocity_msg.twist.angular.z = 0.0
            print('steady level flight')
        self.velocity_pub.publish(velocity_msg)

##############

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
        for i in range(50):
        # for i in range(100):   
            if(rospy.is_shutdown()):
                break
            self.flag = True
            self.steady_level_flight(18.0)
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

    def check_bounds(self, predicted_velocity, predicted_load_factor, velocities_lists, calc_load_factor_lists):
        num_lines_crossed = 0
        print(f'Predicted velocity: {predicted_velocity:.4}, predicted load factor: {predicted_load_factor:.4}')
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
                print(f'taking off')
                # self.takeoff(50.0)
            if current_state.mode == self.mode:
                self.num_lines_crossed = self.check_bounds(self.cb_predict_v_list[-1], self.cb_predict_n_list[-1], self.static_bounds_vel, self.static_bounds_n)
                if self.num_lines_crossed > 0:
                    if self.num_lines_crossed == 1:
                        rospy.loginfo(f"Drone approaching flight envelope bound (level {self.num_lines_crossed}): Mild")
                    elif self.num_lines_crossed == 2:
                        rospy.logwarn(f"ALERT: Drone is approaching flight envelope bound (level {self.num_lines_crossed}): Moderate")
                    elif self.num_lines_crossed == 3:
                        rospy.logerr(f"CAUTION: Drone is breaching flight envelope bound (level {self.num_lines_crossed}): Severe")
                    elif self.num_lines_crossed == 4 or self.num_lines_crossed == 5:
                        rospy.logfatal(f"SUPERVISOR TAKNG OVER (level {self.num_lines_crossed}): Critical")
                        while self.num_lines_crossed >= 4:
                            self.flag = True
                            self.steady_level_flight(18.0)
                            self.num_lines_crossed = self.check_bounds(self.cb_predict_v_list[-1], self.cb_predict_n_list[-1], self.static_bounds_vel, self.static_bounds_n)                        
                            self.rate.sleep()
                            print(f"while loop #: {self.num_lines_crossed}")
                else:
                    self.flag = False
                    self.steady_level_flight(18.0)
                    rospy.loginfo("Drone is within flight envelope")

            last_req = rospy.Time.now()
            self.rate.sleep()
        self.close_csv()

def sigint_handler(sig, frame):
    rospy.loginfo("stopping")
    supervisor.shutdown(18.0)
    rospy.sleep(1)
    sys.exit(0)

if __name__ == "__main__":

    rospy.init_node("Flight_Envelope_Supervisor")
    rate = rospy.Rate(20)

    supervisor = FlightEnvelopeSupervisor()
    
    signal.signal(signal.SIGINT, sigint_handler)

    supervisor.pre_bake_commands()
    supervisor.set_mode()
    supervisor.arm_drone()
    supervisor.steady_level_flight(18.0)
    supervisor.run()

