#!/usr/bin/env python3

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import VFR_HUD
from sensor_msgs.msg import Imu
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Vector3

import config
from helper_functions import quaternionToEuler

class Visualiser:
    '''
    The Visualizer class plots the rolly, pitch, yaw data live and overlays a static plot to compare real-time
    data vs. bounds
    '''
    def __init__(self):
        bounds = FlightEnvelopeAssessment(config.A0, config.CLA, config.CDA, config.ALPHA_STALL, config.WINGAREA, config.AIR_DENSITY, config.MASS, config.G, config.CLA_STALL, config.CDA_STALL)
        self.start_time = time.time()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.velocity = 0.0
        self.vertical_acceleration = 0.0

        self.load_factor = 0.0
        self.load_factor_predict = 0.0
        self.previous_filtered_load_factor = None
        self.weight = .1
        
        self.time_list = []
        self.roll_list = []
        self.pitch_list = []
        self.yaw_list = []
        self.roll_rate_list = []
        self.pitch_rate_list = []
        self.yaw_rate_list = []
        self.velocity_list = []
        self.vertical_acceleration_list = []

        self.load_factor_list = []
        self.load_factor_prediction_list = []

        subPosition = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.position_cb)
        subvfr_hud = rospy.Subscriber('mavros/vfr_hud', VFR_HUD, self.velocity_cb)
        subAcceleration = rospy.Subscriber('mavros/imu/data', Imu, self.az_callback)
        subRates = rospy.Subscriber('mavros/imu/data', Imu, self.rates_cb)

        self.fig, self.ax = plt.subplots()
        self.colors = ['blue', 'red']
        self.labels = ['Load Factor Current', 'prediction']

        self.lines = [self.ax.plot([], [], label=label, color=color, marker='o', linestyle='', markersize=4)[0] for label, color in zip(self.labels, self.colors)]
    
        static_velocity, static_load_factors = bounds.calc_load_factor_vs_velocity_static()
        self.static_plot(static_velocity, static_load_factors)


    def plot_init(self):
        self.ax.set_xlim(left=0, right=25)
        self.ax.set_ylim([-5, 10])
        return self.lines

    def velocity_cb(self, msg):
        self.velocity = msg.groundspeed
        self.velocity_list.append(self.velocity)

    def position_cb(self, msg):
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.roll, self.pitch, self.yaw = quaternionToEuler(qx, qy, qz, qw)
        self.roll_list.append(self.roll)
        self.pitch_list.append(self.pitch)
        self.yaw_list.append(self.yaw)

    def rates_cb(self, msg):
        self.roll_rate = msg.angular_velocity.x
        self.pitch_rate = msg.angular_velocity.y
        self.yaw_rate = msg.angular_velocity.z
        self.roll_rate_list.append(self.roll_rate)
        self.pitch_rate_list.append(self.pitch_rate)
        self.yaw_rate_list.append(self.yaw_rate)
        # print(f"roll_rate: {self.roll_rate:.4} | pitch_rate: {self.pitch_rate:.4} | yaw_rate: {self.yaw_rate:.4}")

    def az_callback(self, msg):
        az = msg.linear_acceleration.z
        self.vertical_acceleration = az
        self.vertical_acceleration_list.append(self.vertical_acceleration)
        self.time_list.append(time.time() - self.start_time)

    def calc_load_factor(self):
        load_factor = self.vertical_acceleration / bounds.g
        self.load_factor_list.append(load_factor)
        self.load_factor = load_factor
        return load_factor

    def predict_next_load_factor(self, velocity_list, time_list):
        v_final = velocity_list[-1]
        v_initial = velocity_list[-2]
        dt = time_list[-1] - time_list[-2]
        delta_v = (v_final - v_initial) / dt
        prediction = delta_v / bounds.g
        return prediction

    def first_order_filter(self, raw_value, previous_filtered_value, weight):
        if previous_filtered_value is None:
            filtered_value = raw_value
        else:
            filtered_value = weight * raw_value + (1 - weight) * previous_filtered_value
        return filtered_value

    def update_plot(self, frame):
        # for vel, roll in zip(self.thinned_velocity_list, self.thinned_pitch_angle_list):
        #     load_factor = self.calc_load_factor(vel, roll)
        #     self.thinned_load_factor_list.append(load_factor)
        #     print(f"velocity: {vel:.4} | roll1: {roll:.4} | n: {load_factor:.4}")
        #     print(f"load_factor: {load_factor:.4}")

        for lol in range(0, 10):
            load_factor = self.calc_load_factor()
            self.load_factor_list.append(load_factor)

            next_load_factor = self.predict_next_load_factor(self.velocity_list, self.time_list)
            filtered_load_factor = self.first_order_filter(next_load_factor, self.previous_filtered_load_factor, self.weight)
            self.previous_filtered_load_factor = filtered_load_factor
            self.load_factor_prediction_list.append(filtered_load_factor + self.load_factor)
            
        print(f'true n:            {load_factor:.4}')
        print(f'predicted n:       {next_load_factor:.4}')
        print(f'filtered n:        {filtered_load_factor:.4}')
        print(f'filtered n + true: {filtered_load_factor + self.load_factor:.4}')


        self.thinned_velocity_list = self.velocity_list[-10:]
        self.thinned_load_factor_list = self.load_factor_list[-10:]
        self.thinned_vertical_acceleration_list = self.vertical_acceleration_list[-10:]
        self.thinned_time_list = self.time_list[-10:]
        self.thinned_load_factor_prediction_list = self.load_factor_prediction_list[-10:]
        
        # print(self.thinned_load_factor_prediction_list[-1])

        self.lines[0].set_data(self.thinned_velocity_list, self.thinned_load_factor_list)
        self.lines[1].set_data(self.thinned_velocity_list[-10:], self.load_factor_prediction_list[-10:])

        print(f"difference:        {self.thinned_load_factor_prediction_list[-1] - self.thinned_load_factor_list[-1]:.4}")
        return self.lines
 
    def static_plot(self, static_velocity, static_load_factors):        
        line_styles = ['-', '-.', '--', '--', ':']  # Define different line styles for each weight
        line_labels = ['clMax', 'Notify', 'Alert', 'Caution', 'Warning', 'uh']  # Define the labels for each line
        line_colors = ['red', 'orange', 'yellow', 'green', 'blue', 'purple']  # Define the colors for each line
        for load_factor, line_style, label, line_color in zip(static_load_factors, line_styles, line_labels, line_colors):
            self.ax.plot(static_velocity, load_factor, color=line_color, alpha=0.3, linestyle=line_style, label=label)

        self.ax.axhline(y=1, color='green', linestyle='--', alpha=0.3)
        self.ax.axhline(y=-1, color='green', linestyle='--', alpha=0.3)
        ticks = len(static_velocity)
        self.ax.set_xticks(range(0, ticks, 5))
        self.ax.set_yticks(range(-ticks, ticks, 1))
        self.ax.grid(visible=True)
        self.ax.legend()


class FlightEnvelopeAssessment():
    '''
        The Flight Envelope Assessment class takes in a models data and it will define the bounds/limits
        of the flight envelope which will then be fed into the supervisor which will set the bounds 
        of the aircraft
    '''
    def __init__(self, A0, CLA, CDA, ALPHA_STALL, WINGAREA, AIR_DENSITY, MASS, G, CLA_STALL, CDA_STALL):
        self.alpha0 = A0
        self.cla = CLA
        self.cla_stall = CLA_STALL
        self.cda = CDA
        self.cda_stall = CDA_STALL
        self.alpha_stall = ALPHA_STALL
        self.area = WINGAREA
        self.rho = AIR_DENSITY
        self.mass = MASS
        self.g = G

        # calculated
        self.coefficient_lift = 0.0
        self.dynamic_pressure = 0.01
        self.lift = 0.0
        self.velocity = 0.0
        self.load_factor = 0.0
        self.vStall = 0.0

        self.coefficient_lift_list = []
        self.angleList = np.arange(0, ALPHA_STALL, 0.01 * np.pi/180)

        for angle in self.angleList:
            self.coefficient_lift = self.calc_cl(angle)
            self.coefficient_lift_list.append(self.coefficient_lift)

        self.clMax = max(self.coefficient_lift_list)
        self.clMaxWeights = [.5, .6, .7, .8,]

        self.calc_v_stall()
        self.angleListDegrees = np.rad2deg(self.angleList)

    def calc_cl(self, angle_of_attack):
        clift = self.cla * (angle_of_attack - self.alpha0)
        # print(f"Coeff. Lift: {clift}")
        return clift

    def calc_lift(self, velocity, AoA):
        lift = (self.calc_cl(AoA) * (self.rho * (velocity**2) * .5) * self.area)
        return lift
    
    def calc_v_stall(self):
        self.vStall = np.sqrt((2 * self.mass * self.g) / (self.rho * self.area * self.clMax))

    def calc_load_factor_vs_velocity_static(self):
        velocities = np.linspace(0, 100, 250)
        calc_load_factor_lists = []

        # Calculate load factors for CLmax without any weight adjustments
        calc_load_factor_list = []
        for v in velocities:
            self.dynamic_pressure = 0.5 * self.rho * v ** 2
            self.lift = self.clMax * self.area * self.dynamic_pressure
            self.load_factor = self.lift / (self.mass * self.g)
            calc_load_factor_list.append(self.load_factor)
        calc_load_factor_lists.append(calc_load_factor_list)  # Add CLmax list to the list of load factor lists

        # Calculate load factors for CLmax with weight adjustments
        for weight in self.clMaxWeights:
            calc_load_factor_list = []
            for v in velocities:
                self.dynamic_pressure = 0.5 * self.rho * v ** 2
                self.lift = (self.clMax * weight) * self.area * self.dynamic_pressure
                self.load_factor = self.lift / (self.mass * self.g)
                calc_load_factor_list.append(self.load_factor)
            calc_load_factor_lists.append(calc_load_factor_list)  # Add weighted CLmax list to the list of load factor lists

        return velocities, calc_load_factor_lists



if __name__ == "__main__":

    rospy.init_node('flight_envelope_assessment_node')
    plt.close("all")
    rate = rospy.Rate(20)
    time_zero = rospy.get_time()
    bounds = FlightEnvelopeAssessment(config.A0, config.CLA, config.CDA, config.ALPHA_STALL, config.WINGAREA, config.AIR_DENSITY, config.MASS, config.G, config.CLA_STALL, config.CDA_STALL)
    vis = Visualiser()

    try:
        while not rospy.is_shutdown():
            ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, frames=1, blit=False)
            plt.show(block=True)
            rate.sleep()
        plt.close('all')
    except KeyboardInterrupt:
        print("Exiting Flight Envelope Assessment")

